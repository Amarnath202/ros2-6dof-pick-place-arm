#!/usr/bin/env python3
# ============================================================
# arm_vision/arm_vision/color_detector.py
#
# OpenCV Color-based object detector (Red/Blue cubes).
# Replaces YOLO for more reliable simulation performance.
# ============================================================

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        self.get_logger().info('=== Color Detector Node Starting ===')

        self.declare_parameter('target_color', 'red') # 'red' or 'blue'
        self.declare_parameter('image_topic', '/overhead_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/overhead_camera/camera_info')
        self.declare_parameter('camera_frame', 'overhead_camera_link')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('object_height_assumption', 0.05)

        self.target_color = self.get_parameter('target_color').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.object_height = self.get_parameter('object_height_assumption').value

        self.bridge = CvBridge()
        self.camera_info = None

        self._cam_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._cam_info_callback, 10
        )
        self._image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 10
        )

        self._pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        
        if self.publish_annotated:
            self._annotated_pub = self.create_publisher(Image, '/vision/annotated_image', 10)

        self.get_logger().info(f'Configured to detect {self.target_color.upper()} objects.')

    def _cam_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _image_callback(self, msg: Image):
        if self.camera_info is None:
            self.get_logger().warn('Waiting for CameraInfo...', throttle_duration_sec=2.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = None

        if self.target_color.lower() == 'red':
            # Red wraps around in HSV
            lower1 = np.array([0, 120, 70])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([170, 120, 70])
            upper2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower1, upper1)
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = mask1 + mask2
        elif self.target_color.lower() == 'blue':
            lower = np.array([100, 150, 0])
            upper = np.array([140, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            self.get_logger().error(f'Unknown color {self.target_color}')
            return

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:  # Min area threshold
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    # Draw on image
                    cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                    cv2.putText(cv_image, f"{self.target_color.upper()} Cube", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    # Compute 3D pose
                    self._publish_pose(cX, cY, msg.header.stamp)

        if self.publish_annotated:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header
            self._annotated_pub.publish(out_msg)

    def _publish_pose(self, u, v, stamp):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        # The camera is at z=1.2, table is at z=0.45.
        # But this node publishes in the camera's frame.
        # depth (z in camera frame) = 1.2 - (0.45 + object_height/2)
        # We can just assume a fixed depth.
        depth = 1.2 - (0.45 + self.object_height / 2.0)

        # 3D coordinates in camera frame (Z forward, X right, Y down)
        x_c = (u - cx) * depth / fx
        y_c = (v - cy) * depth / fy
        z_c = depth

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame
        
        pose_msg.pose.position.x = float(x_c)
        pose_msg.pose.position.y = float(y_c)
        pose_msg.pose.position.z = float(z_c)
        
        # Identity orientation
        pose_msg.pose.orientation.w = 1.0
        
        self._pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
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
