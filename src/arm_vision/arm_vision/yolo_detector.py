#!/usr/bin/env python3
# ============================================================
# arm_vision/arm_vision/yolo_detector.py
#
# YOLOv8 Object Detection ROS2 Node.
#
# Subscribes to camera image topic, runs YOLOv8 inference,
# and publishes:
#   - Annotated image with bounding boxes
#   - Detection results as vision_msgs/Detection2DArray
#   - Cropped ROI images of each detection (for EasyOCR)
#   - 3D pose estimate of detected object (for pick-place)
#
# SUBSCRIBED TOPICS:
#   /wrist_camera/image_raw      (sensor_msgs/Image) — camera feed
#   /wrist_camera/camera_info    (sensor_msgs/CameraInfo) — intrinsics
#
# PUBLISHED TOPICS:
#   /yolo/image_annotated        (sensor_msgs/Image) — visualization
#   /yolo/detections             (vision_msgs/Detection2DArray)
#   /yolo/detection_crop         (sensor_msgs/Image) — ROI for OCR
#   /detected_object_pose        (geometry_msgs/PoseStamped) — 3D pose
#
# PARAMETERS:
#   model_name: YOLO model file (default: yolov8n.pt)
#   confidence: Detection confidence threshold (default: 0.5)
#   device: 'cpu' or 'cuda' or 'cuda:0'
#   target_class: Specific class to track (-1 = all)
#   image_topic: Input image topic
#   camera_frame: Camera TF frame name
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import math
from typing import Optional, List, Tuple

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2DArray, Detection2D, ObjectHypothesisWithPose,
    BoundingBox2D, Pose2D
)

# OpenCV bridge for ROS2 ↔ OpenCV conversion
from cv_bridge import CvBridge, CvBridgeError

# YOLO — lazy import to avoid startup delays
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print('WARNING: ultralytics not installed. Run: pip3 install ultralytics')


class YOLODetectorNode(Node):
    """
    YOLOv8 object detection node.

    Processes camera images, detects objects, and publishes
    3D pose estimates for the pick-and-place pipeline.
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.get_logger().info('=== YOLO Detector Node Starting ===')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('target_class', -1)   # -1 = all classes
        self.declare_parameter('target_class_name', 'bottle')  # specific class
        self.declare_parameter('image_topic', '/wrist_camera/image_raw')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('publish_crop', True)
        self.declare_parameter('object_height_assumption', 0.06)  # 6cm box

        self.model_name = self.get_parameter('model_name').value
        self.confidence = self.get_parameter('confidence').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.device = self.get_parameter('device').value
        self.target_class = self.get_parameter('target_class').value
        self.target_class_name = self.get_parameter('target_class_name').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.publish_crop = self.get_parameter('publish_crop').value
        self.object_height = self.get_parameter('object_height_assumption').value

        # ── State ────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._model: Optional[YOLO] = None
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: Optional[np.ndarray] = None
        self._camera_info_received = False
        self._detection_count = 0

        # ── QoS profiles ─────────────────────────────────────────────────
        # Best effort for images (don't block on missed frames)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Reliable for detections
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self._image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            image_qos
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            '/wrist_camera/camera_info',
            self._camera_info_callback,
            10
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._annotated_pub = self.create_publisher(
            Image, '/yolo/image_annotated', 10
        )

        self._detections_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', reliable_qos
        )

        self._crop_pub = self.create_publisher(
            Image, '/yolo/detection_crop', 10
        )

        self._object_pose_pub = self.create_publisher(
            PoseStamped, '/detected_object_pose', reliable_qos
        )

        self._status_pub = self.create_publisher(
            String, '/yolo/status', 10
        )

        # ── Load YOLO model ──────────────────────────────────────────────
        self._load_model()

        # Status timer
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Model: {self.model_name}, Confidence: {self.confidence}')
        self.get_logger().info('YOLO Detector ready.')

    def _load_model(self):
        """Load the YOLOv8 model. Downloads if not cached."""
        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics not available!')
            return

        self.get_logger().info(f'Loading YOLO model: {self.model_name}...')
        try:
            # YOLO will auto-download the model if not found locally
            self._model = YOLO(self.model_name)
            self._model.to(self.device)

            # Warm up the model with a dummy inference
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self._model.predict(dummy, verbose=False)

            self.get_logger().info(
                f'YOLO model loaded! Classes: {len(self._model.names)}'
            )
            self.get_logger().info(
                f'Running on device: {self.device}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self._model = None

    def _camera_info_callback(self, msg: CameraInfo):
        """
        Store camera intrinsic parameters for 3D projection.
        Only processes once (camera parameters don't change).
        """
        if self._camera_info_received:
            return

        # Camera matrix K: [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        self._camera_matrix = np.array(msg.k).reshape(3, 3)
        self._dist_coeffs = np.array(msg.d)
        self._image_width = msg.width
        self._image_height = msg.height

        self._camera_info_received = True

        self.get_logger().info(
            f'Camera info received: {msg.width}×{msg.height}'
        )
        self.get_logger().info(
            f'  fx={self._camera_matrix[0,0]:.1f}, '
            f'fy={self._camera_matrix[1,1]:.1f}, '
            f'cx={self._camera_matrix[0,2]:.1f}, '
            f'cy={self._camera_matrix[1,2]:.1f}'
        )

    def _image_callback(self, msg: Image):
        """
        Main callback: process each camera frame with YOLO.
        """
        if self._model is None:
            return

        # Convert ROS Image to OpenCV
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Run YOLO inference
        try:
            results = self._model.predict(
                cv_image,
                conf=self.confidence,
                iou=self.iou_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().warn(f'YOLO inference error: {e}')
            return

        # Process results
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        detections_msg.header.frame_id = self.camera_frame

        annotated_image = cv_image.copy()
        best_detection = None
        best_confidence = 0.0

        if results and len(results) > 0:
            result = results[0]  # batch size = 1

            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes

                for i, box in enumerate(boxes):
                    # Bounding box coordinates (xyxy format)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self._model.names[cls_id]

                    # Filter by target class if specified
                    if (self.target_class >= 0 and cls_id != self.target_class):
                        continue

                    # Build Detection2D message
                    detection = Detection2D()
                    detection.header = msg.header

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = cls_name
                    hyp.hypothesis.score = conf
                    detection.results.append(hyp)

                    # Bounding box
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    width = x2 - x1
                    height = y2 - y1

                    bbox = BoundingBox2D()
                    bbox.center.position.x = cx
                    bbox.center.position.y = cy
                    bbox.size_x = width
                    bbox.size_y = height
                    detection.bbox = bbox

                    detections_msg.detections.append(detection)

                    # Track best (highest confidence) detection
                    if conf > best_confidence:
                        best_confidence = conf
                        best_detection = {
                            'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                            'cx': cx, 'cy': cy,
                            'width': width, 'height': height,
                            'conf': conf, 'class': cls_name, 'cls_id': cls_id
                        }

                    # Draw bounding box on annotated image
                    color = self._get_class_color(cls_id)
                    cv2.rectangle(
                        annotated_image,
                        (int(x1), int(y1)), (int(x2), int(y2)),
                        color, 2
                    )
                    label = f'{cls_name} {conf:.2f}'
                    cv2.putText(
                        annotated_image, label,
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                    )

                    # Draw center point
                    cv2.circle(
                        annotated_image,
                        (int(cx), int(cy)), 5, (0, 255, 0), -1
                    )

        # Publish detections
        self._detections_pub.publish(detections_msg)
        self._detection_count += 1

        # Publish annotated image
        if self.publish_annotated:
            # Add detection count overlay
            cv2.putText(
                annotated_image,
                f'Detections: {len(detections_msg.detections)}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (255, 255, 0), 2
            )
            try:
                annotated_msg = self._bridge.cv2_to_imgmsg(
                    annotated_image, 'bgr8'
                )
                annotated_msg.header = msg.header
                self._annotated_pub.publish(annotated_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f'Failed to publish annotated image: {e}')

        # Process best detection
        if best_detection is not None:
            self._process_best_detection(cv_image, best_detection, msg.header)

    def _process_best_detection(
        self,
        image: np.ndarray,
        detection: dict,
        header
    ):
        """
        Process the best (highest confidence) detection:
          1. Crop and publish ROI for EasyOCR
          2. Estimate 3D pose and publish
        """
        x1, y1 = int(detection['x1']), int(detection['y1'])
        x2, y2 = int(detection['x2']), int(detection['y2'])

        # ── Publish crop for EasyOCR ─────────────────────────────────────
        if self.publish_crop:
            # Add some padding around the bounding box
            pad = 10
            h, w = image.shape[:2]
            crop_x1 = max(0, x1 - pad)
            crop_y1 = max(0, y1 - pad)
            crop_x2 = min(w, x2 + pad)
            crop_y2 = min(h, y2 + pad)

            crop = image[crop_y1:crop_y2, crop_x1:crop_x2]

            try:
                crop_msg = self._bridge.cv2_to_imgmsg(crop, 'bgr8')
                crop_msg.header = header
                self._crop_pub.publish(crop_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f'Failed to publish crop: {e}')

        # ── Estimate 3D pose ──────────────────────────────────────────────
        pose = self._estimate_3d_pose(
            detection['cx'], detection['cy'],
            detection['width'], detection['height']
        )

        if pose is not None:
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.header.frame_id = self.camera_frame
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.position.z = pose[2]
            pose_msg.pose.orientation.w = 1.0

            self._object_pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'Object [{detection["class"]}] at camera coords: '
                f'({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})'
            )

    def _estimate_3d_pose(
        self,
        px: float, py: float,
        bbox_w: float, bbox_h: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        Estimate 3D position from 2D bounding box center using
        pinhole camera model.

        Assumptions:
          - Object sits on a known table surface (z_world known)
          - We use the bounding box size to estimate distance

        Args:
            px, py: Bounding box center in pixels
            bbox_w, bbox_h: Bounding box dimensions in pixels

        Returns:
            (x, y, z) in camera frame, or None if camera info not available
        """
        if not self._camera_info_received:
            self.get_logger().warn(
                'No camera info yet — cannot estimate 3D pose',
                throttle_duration_sec=5.0
            )
            return None

        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        cx = self._camera_matrix[0, 2]
        cy = self._camera_matrix[1, 2]

        # Estimate depth using known object size
        # Z_camera = (real_size * focal_length) / pixel_size
        # For a 6cm box:
        real_object_size = self.object_height  # meters
        pixel_size = max(bbox_w, bbox_h)  # use larger dimension

        if pixel_size < 5:  # avoid division by zero
            return None

        # Estimated depth (distance from camera to object center)
        z_camera = (real_object_size * fx) / pixel_size

        # Back-project 2D pixel to 3D camera-frame point
        x_camera = (px - cx) * z_camera / fx
        y_camera = (py - cy) * z_camera / fy

        return (x_camera, y_camera, z_camera)

    def _get_class_color(self, class_id: int) -> tuple:
        """Return a distinct BGR color for each class ID."""
        colors = [
            (0, 255, 0),    # green
            (255, 0, 0),    # blue
            (0, 0, 255),    # red
            (255, 255, 0),  # cyan
            (255, 0, 255),  # magenta
            (0, 255, 255),  # yellow
            (128, 0, 255),  # purple
            (255, 128, 0),  # orange
        ]
        return colors[class_id % len(colors)]

    def _publish_status(self):
        """Publish YOLO status information."""
        msg = String()
        model_status = 'loaded' if self._model is not None else 'not_loaded'
        msg.data = (
            f'YOLO status: model={model_status}, '
            f'camera_info={self._camera_info_received}, '
            f'detections_processed={self._detection_count}'
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
