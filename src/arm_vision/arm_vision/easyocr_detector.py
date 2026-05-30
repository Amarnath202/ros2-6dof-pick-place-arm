#!/usr/bin/env python3
# ============================================================
# arm_vision/arm_vision/easyocr_detector.py
#
# EasyOCR Text Detection ROS2 Node.
#
# Subscribes to cropped object images from YOLO, runs EasyOCR
# inference, and publishes detected text.
#
# This allows the robot to:
#   - Read labels on boxes
#   - Identify which object to pick based on text
#   - Read destination labels for placing
#
# SUBSCRIBED TOPICS:
#   /yolo/detection_crop  (sensor_msgs/Image) — cropped object image
#
# PUBLISHED TOPICS:
#   /ocr/detected_text      (std_msgs/String) — raw detected text
#   /ocr/image_annotated    (sensor_msgs/Image) — text overlay image
#   /ocr/result             (std_msgs/String) — JSON with all detections
#
# PARAMETERS:
#   languages: EasyOCR language list (default: ['en'])
#   confidence_threshold: Minimum OCR confidence (default: 0.5)
#   gpu: Use GPU for OCR (default: false)
#   target_text: Text to look for (triggers pick-place event)
# ============================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import json
import time
from typing import List, Tuple, Optional

# ROS2 messages
from sensor_msgs.msg import Image
from std_msgs.msg import String

# OpenCV bridge
from cv_bridge import CvBridge, CvBridgeError

# EasyOCR — lazy import
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False
    print('WARNING: easyocr not installed. Run: pip3 install easyocr')


class EasyOCRDetectorNode(Node):
    """
    EasyOCR text recognition node.

    Processes cropped images from YOLO detector and reads
    any text present on detected objects.
    """

    def __init__(self):
        super().__init__('easyocr_detector_node')

        self.get_logger().info('=== EasyOCR Detector Node Starting ===')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('languages', ['en'])
        self.declare_parameter('confidence_threshold', 0.50)
        self.declare_parameter('gpu', False)
        self.declare_parameter('target_text', '')      # Text to search for
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('min_text_length', 2)  # Minimum chars to report

        self.languages = self.get_parameter('languages').value
        self.confidence = self.get_parameter('confidence_threshold').value
        self.use_gpu = self.get_parameter('gpu').value
        self.target_text = self.get_parameter('target_text').value.lower()
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.min_text_length = self.get_parameter('min_text_length').value

        # ── State ────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._reader: Optional[easyocr.Reader] = None
        self._last_text = ''
        self._detection_count = 0

        # Throttle: don't run OCR on every frame (expensive)
        self._last_ocr_time = 0.0
        self._ocr_interval = 0.5  # seconds between OCR runs

        # ── QoS ──────────────────────────────────────────────────────────
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self._crop_sub = self.create_subscription(
            Image,
            '/yolo/detection_crop',
            self._crop_callback,
            image_qos
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._text_pub = self.create_publisher(
            String, '/ocr/detected_text', 10
        )

        self._annotated_pub = self.create_publisher(
            Image, '/ocr/image_annotated', 10
        )

        self._result_pub = self.create_publisher(
            String, '/ocr/result', 10
        )

        self._status_pub = self.create_publisher(
            String, '/ocr/status', 10
        )

        # ── Initialize EasyOCR ───────────────────────────────────────────
        self._init_reader()

        self.get_logger().info('EasyOCR Detector ready.')
        if self.target_text:
            self.get_logger().info(f'Looking for text: "{self.target_text}"')

    def _init_reader(self):
        """Initialize EasyOCR reader (downloads models on first run)."""
        if not EASYOCR_AVAILABLE:
            self.get_logger().error('easyocr not available!')
            return

        self.get_logger().info(
            f'Initializing EasyOCR with languages: {self.languages}...'
        )
        self.get_logger().info('(First run downloads models — may take a minute)')

        try:
            self._reader = easyocr.Reader(
                self.languages,
                gpu=self.use_gpu,
                verbose=False
            )
            self.get_logger().info('EasyOCR reader initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize EasyOCR: {e}')
            self._reader = None

    def _crop_callback(self, msg: Image):
        """
        Process a cropped image from YOLO.
        Throttled to avoid running OCR on every frame.
        """
        if self._reader is None:
            return

        # Throttle OCR
        current_time = time.time()
        if current_time - self._last_ocr_time < self._ocr_interval:
            return
        self._last_ocr_time = current_time

        # Convert to OpenCV
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Preprocess image for better OCR
        processed = self._preprocess_for_ocr(cv_image)

        # Run EasyOCR
        try:
            results = self._reader.readtext(
                processed,
                detail=1,          # Return full detail (bbox + text + conf)
                paragraph=False,   # Don't merge paragraphs
            )
        except Exception as e:
            self.get_logger().warn(f'EasyOCR error: {e}')
            return

        # Process results
        detected_texts = []
        annotated = cv_image.copy()

        for (bbox, text, confidence) in results:
            if confidence < self.confidence:
                continue
            if len(text.strip()) < self.min_text_length:
                continue

            detected_texts.append({
                'text': text.strip(),
                'confidence': round(float(confidence), 3),
                'bbox': [[int(p[0]), int(p[1])] for p in bbox]
            })

            # Draw on annotated image
            pts = np.array(bbox, dtype=np.int32)
            cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)

            # Background for text label
            label = f'{text} ({confidence:.2f})'
            (lw, lh), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            top_left = (int(bbox[0][0]), int(bbox[0][1]) - lh - 5)
            cv2.rectangle(
                annotated,
                top_left,
                (top_left[0] + lw, top_left[1] + lh + 5),
                (0, 255, 0), -1
            )
            cv2.putText(
                annotated, label,
                (int(bbox[0][0]), int(bbox[0][1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
            )

        # Publish results
        self._detection_count += 1

        # Publish all text combined
        if detected_texts:
            combined_text = ' '.join([d['text'] for d in detected_texts])
            self._last_text = combined_text

            # Publish raw text
            text_msg = String()
            text_msg.data = combined_text
            self._text_pub.publish(text_msg)

            # Publish JSON result
            result_msg = String()
            result_msg.data = json.dumps({
                'timestamp': current_time,
                'detections': detected_texts,
                'combined_text': combined_text
            })
            self._result_pub.publish(result_msg)

            self.get_logger().info(
                f'OCR detected: "{combined_text}" '
                f'({len(detected_texts)} regions)'
            )

            # Check for target text
            if self.target_text and self.target_text in combined_text.lower():
                self.get_logger().info(
                    f'TARGET TEXT FOUND: "{self.target_text}" in "{combined_text}"'
                )
                # Could trigger additional actions here

        # Publish annotated image
        if self.publish_annotated:
            # Add OCR status overlay
            status = f'OCR: {len(detected_texts)} text regions found'
            cv2.putText(
                annotated, status,
                (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 0), 1
            )

            try:
                ann_msg = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
                ann_msg.header = msg.header
                self._annotated_pub.publish(ann_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f'Failed to publish annotated: {e}')

    def _preprocess_for_ocr(self, image: np.ndarray) -> np.ndarray:
        """
        Preprocess image to improve OCR accuracy.
        Steps:
          1. Upscale if too small
          2. Convert to grayscale
          3. Apply CLAHE contrast enhancement
          4. Denoise
        """
        h, w = image.shape[:2]

        # Upscale small images (EasyOCR works better with larger images)
        min_size = 200
        if h < min_size or w < min_size:
            scale = max(min_size / h, min_size / w)
            new_w = int(w * scale)
            new_h = int(h * scale)
            image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

        # Convert to grayscale for processing
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        # Denoise
        denoised = cv2.fastNlMeansDenoising(enhanced, h=10)

        # Convert back to BGR for EasyOCR
        result = cv2.cvtColor(denoised, cv2.COLOR_GRAY2BGR)

        return result

    def get_last_detected_text(self) -> str:
        """Return the most recently detected text."""
        return self._last_text


def main(args=None):
    rclpy.init(args=args)
    node = EasyOCRDetectorNode()
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
