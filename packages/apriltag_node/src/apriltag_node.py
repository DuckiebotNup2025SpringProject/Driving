#!/bin/python3
import os

import apriltag
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class AprilTagNode(Node):
    def __init__(self, bot_name=None):
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        super().__init__('apriltag_node')
        self.bot_name = bot_name
        self.apriltag_topic = f'/{bot_name}/image/compressed'

        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.apriltag_topic,
            self.parse_apriltag,
            10)

        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Publishing to {self.apriltag_topic}')
        self.actual_image = 0

        self.timer = self.create_timer(5, lambda: self.get_result(self.actual_image))

    def parse_apriltag(self, msg):
        try:
            # Convert the bytes to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the image from JPEG/PNG format into standard BGR format
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                self.get_logger().info("Unable to decode the image")
                return

            self.actual_image = img.copy()

        except Exception:
            self.get_logger().error("Error parsing image")

    def get_result(self, image):
        try:
            # Convert the image to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Create an AprilTag detector
            detector = apriltag.Detector()

            # Detect the tags in the image
            results = detector.detect(gray)

            if not results:
                self.get_logger().info("No tags detected")
                return

            focal_length_mm = 300
            tag_size_mm = 65

            # Display information about the detected tags
            for tag in results:
                self.get_logger().info("------------------------------------------------")
                self.get_logger().info("Tag Id:" + str(tag.tag_id))
                tag_width_pixels = np.linalg.norm(tag.corners[0] - tag.corners[3])
                distance_mm = (focal_length_mm * tag_size_mm) / tag_width_pixels
                self.get_logger().info("Distance to tag: " + str(distance_mm) + " mm")

            self.get_logger().info("Detection completed successfully")

        except Exception:
            self.get_logger().error("Error obtaining detection results")


def main(args=None):
    rclpy.init(args=args)
    image_processor = AprilTagNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
