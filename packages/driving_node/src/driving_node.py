#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse

# TODO: find schema for WheelCmd.msg
#       from <path_to_msg_dir>.msg import WheelCmd


class SegmentationNode(Node):
    def __init__(self, bot_name=None):
        # Initialisation of the Node
        if bot_name is None:
            self.get_logger().error('Bot name not specified')
        super().__init__('driving_node')
        self.bot_name = bot_name
        self.motor_topic = f'/{bot_name}/wheels_cmd'
        self.segmentation_topic = f'/{bot_name}/mask'

        # Set up the image subscriber
        try:
            self.image_subscription = self.create_subscription(
                Image,
                self.segmentation_topic,
                self.image_callback,
                10)
        except Exception as e:
            self.get_logger().error(f'Error while subscribing to mask: {str(e)}')

        # Set up the motors publisher
        try:
            self.motor_publisher = self.create_publisher(
                Float32MultiArray,
                self.motor_topic,
                10)
        except Exception as e:
            self.get_logger().error(f'Error while creating a motor publisher: {str(e)}')

        # CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Logs about node start up
        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Subscribed to {self.segmentation_topic}')
        self.get_logger().info(f'Publishing to {self.motor_topic}')

    def feedback_callback(self, feedback):
        self.get_logger().info('Feedback: {0}'.format(feedback.feedback.sequence))

    def mask_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # Assuming the mask is already a binary image
            mask = self.bridge.imgmsg_to_cv2(msg)

            # Calculate motor control values directly from the mask
            vel_left, vel_right= self.calculate_motor_values(mask)

            # Publish motor control values
            self.motor_publisher(vel_left, vel_right)

        except Exception as e:
            self.get_logger().error(f'Error processing mask: {str(e)}')

    def motor_publisher(self, vel_left, vel_right):
        if not (-1.0 <= vel_left <= 1.0):
            self.get_logger().error(f'Velocity left out of range: {vel_left}')
            vel_left = max(-1.0, min(1.0, vel_left))
        if not (-1.0 <= vel_right <= 1.0):
            self.get_logger().error(f'Velocity right out of range: {vel_right}')
            vel_right = max(-1.0, min(1.0, vel_right))
        motor_msg = WheelCmd()
        motor_msg.vel_left = float(vel_left)
        motor_msg.vel_right = float(vel_right)
        try:
            self.motor_publisher.publish(motor_msg)
            self.get_logger().debug(f'Published motor values: [{left_motor}, {right_motor}]')
        except Exception as e:
            self.get_logger().error(f'Error publishing motor values: {str(e)}')

    def calculate_motor_values(self, mask):
        # TODO: Algorithm for best path.
        pass

    def shutdown(self):
        # TODO: Write shutdown alg.
        pass

def main(args=None):
    rclpy.init(args=args)
    image_processor = SegmentationNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
