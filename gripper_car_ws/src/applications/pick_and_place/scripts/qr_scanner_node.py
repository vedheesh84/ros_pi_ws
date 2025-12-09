#!/usr/bin/env python3
"""
QR Scanner Node
===============
ROS2 service node for QR code detection and decoding.

This node subscribes to a camera image topic and provides a service
to scan for QR codes. When the service is called, it captures the
current image and attempts to detect and decode any QR codes.

Services:
    /qr_scanner/scan (std_srvs/Trigger):
        Scan current camera image for QR codes.
        Returns success=True with QR content in message field.
        Returns success=False if no QR code detected.

Topics:
    Subscribed:
        /camera/image_raw (sensor_msgs/Image): Camera feed

    Published:
        /qr_scanner/detections (std_msgs/String): Continuous QR detections
        /qr_scanner/debug_image (sensor_msgs/Image): Image with QR overlay

Parameters:
    camera_topic (str): Camera image topic to subscribe to
    min_qr_size (int): Minimum QR code size in pixels
    detection_timeout (float): Timeout for detection attempts
    publish_debug (bool): Publish debug image with QR overlay
    continuous_mode (bool): Continuously publish detections

Dependencies:
    - pyzbar: QR code decoding library
    - opencv: Image processing
    - cv_bridge: ROS-OpenCV bridge

Install pyzbar:
    sudo apt install libzbar0
    pip3 install pyzbar
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
import threading


# Try to import pyzbar, provide helpful error if not installed
try:
    from pyzbar import pyzbar
    from pyzbar.pyzbar import Decoded
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False


@dataclass
class QRDetection:
    """
    Represents a detected QR code.

    Attributes:
        data: Decoded QR code content (string)
        rect: Bounding box (x, y, width, height)
        polygon: Corner points of QR code
    """
    data: str
    rect: Tuple[int, int, int, int]
    polygon: List[Tuple[int, int]]


class QRScannerNode(Node):
    """
    ROS2 node providing QR code scanning service.

    The node maintains the latest camera frame and provides a service
    to scan it for QR codes on demand. Optionally can run in continuous
    mode, publishing any detected QR codes.
    """

    def __init__(self):
        super().__init__('qr_scanner_node')

        # Check pyzbar availability
        if not PYZBAR_AVAILABLE:
            self.get_logger().error(
                "pyzbar library not found! Install with:\n"
                "  sudo apt install libzbar0 && pip3 install pyzbar"
            )

        # Declare parameters with defaults
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('min_qr_size', 50)
        self.declare_parameter('detection_timeout', 0.5)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('continuous_mode', False)

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.min_qr_size = self.get_parameter('min_qr_size').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.continuous_mode = self.get_parameter('continuous_mode').value

        # OpenCV bridge for image conversion
        self.bridge = CvBridge()

        # Latest image storage (thread-safe access)
        self._latest_image: Optional[np.ndarray] = None
        self._image_lock = threading.Lock()
        self._last_image_time = None

        # Callback group for concurrent service/subscription handling
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to camera feed
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._image_callback,
            10,
            callback_group=self.callback_group
        )

        # Create scan service
        self.scan_service = self.create_service(
            Trigger,
            '/qr_scanner/scan',
            self._scan_callback,
            callback_group=self.callback_group
        )

        # Publishers
        self.detection_pub = self.create_publisher(String, '/qr_scanner/detections', 10)

        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image, '/qr_scanner/debug_image', 10
            )

        # Continuous mode timer
        if self.continuous_mode:
            self.scan_timer = self.create_timer(
                0.1,  # 10 Hz continuous scanning
                self._continuous_scan_callback
            )

        self.get_logger().info(
            f"QR Scanner initialized:\n"
            f"  Camera topic: {self.camera_topic}\n"
            f"  Min QR size: {self.min_qr_size}px\n"
            f"  Continuous mode: {self.continuous_mode}\n"
            f"  Debug images: {self.publish_debug}"
        )

    def _image_callback(self, msg: Image) -> None:
        """
        Callback for camera image messages.

        Converts ROS Image to OpenCV format and stores for scanning.

        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            with self._image_lock:
                self._latest_image = cv_image
                self._last_image_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def _scan_callback(self, request, response) -> Trigger.Response:
        """
        Service callback to scan current image for QR codes.

        Args:
            request: Empty trigger request
            response: Trigger response with success status and message

        Returns:
            Trigger.Response with QR content or error message
        """
        if not PYZBAR_AVAILABLE:
            response.success = False
            response.message = "pyzbar library not available"
            return response

        # Get current image
        with self._image_lock:
            if self._latest_image is None:
                response.success = False
                response.message = "No camera image available"
                return response

            image = self._latest_image.copy()

        # Attempt QR detection
        detections = self._detect_qr_codes(image)

        if not detections:
            response.success = False
            response.message = "No QR code detected"
            return response

        # Return first detected QR code
        qr_content = detections[0].data
        response.success = True
        response.message = qr_content

        self.get_logger().info(f"QR code detected: '{qr_content}'")

        # Publish debug image if enabled
        if self.publish_debug:
            self._publish_debug_image(image, detections)

        return response

    def _detect_qr_codes(self, image: np.ndarray) -> List[QRDetection]:
        """
        Detect and decode QR codes in an image.

        Uses pyzbar library for detection. Applies preprocessing
        to improve detection rate.

        Args:
            image: OpenCV image (BGR format)

        Returns:
            List of QRDetection objects for each detected QR code
        """
        detections = []

        # Convert to grayscale for better detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Try detection on original image
        decoded_objects = pyzbar.decode(gray)

        # If no detection, try with contrast enhancement
        if not decoded_objects:
            # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            decoded_objects = pyzbar.decode(enhanced)

        # If still no detection, try with threshold
        if not decoded_objects:
            _, binary = cv2.threshold(
                gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
            decoded_objects = pyzbar.decode(binary)

        # Process detections
        for obj in decoded_objects:
            # Filter by minimum size
            rect = obj.rect
            if rect.width < self.min_qr_size or rect.height < self.min_qr_size:
                continue

            # Decode data
            try:
                data = obj.data.decode('utf-8')
            except UnicodeDecodeError:
                data = obj.data.decode('latin-1')

            # Extract polygon points
            polygon = [(point.x, point.y) for point in obj.polygon]

            detections.append(QRDetection(
                data=data,
                rect=(rect.left, rect.top, rect.width, rect.height),
                polygon=polygon
            ))

        return detections

    def _publish_debug_image(
        self, image: np.ndarray, detections: List[QRDetection]
    ) -> None:
        """
        Publish debug image with QR code overlays.

        Draws bounding boxes and decoded content on the image.

        Args:
            image: Original OpenCV image
            detections: List of detected QR codes
        """
        debug_image = image.copy()

        for det in detections:
            # Draw polygon around QR code
            points = np.array(det.polygon, dtype=np.int32)
            cv2.polylines(debug_image, [points], True, (0, 255, 0), 2)

            # Draw text with QR content
            x, y, _, _ = det.rect
            cv2.putText(
                debug_image, det.data,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

        # Convert back to ROS Image and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

    def _continuous_scan_callback(self) -> None:
        """
        Timer callback for continuous mode scanning.

        Scans each frame and publishes any detected QR codes.
        """
        with self._image_lock:
            if self._latest_image is None:
                return
            image = self._latest_image.copy()

        detections = self._detect_qr_codes(image)

        for det in detections:
            msg = String()
            msg.data = det.data
            self.detection_pub.publish(msg)

        if detections and self.publish_debug:
            self._publish_debug_image(image, detections)


def main(args=None):
    """Main entry point for QR scanner node."""
    rclpy.init(args=args)

    node = QRScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
