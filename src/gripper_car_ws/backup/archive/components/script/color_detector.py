#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        # subscribe to camera stream
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # <-- match your topic name
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # convert ROS image → OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------- COLOR RANGES ----------
        # define HSV ranges for 3 colors (adjust as needed)
        colors = {
            "red":   [(0, 120, 70), (10, 255, 255)],
            "blue":  [(94, 80, 2), (126, 255, 255)],
            "green": [(40, 40, 40), (70, 255, 255)]
        }

        for color_name, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            count = cv2.countNonZero(mask)

            if count > 5000:  # pixels threshold
                self.get_logger().info(f"{color_name.upper()} object detected ({count} px)")

        # show window (useful for debugging)
        cv2.imshow("Pi Camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
