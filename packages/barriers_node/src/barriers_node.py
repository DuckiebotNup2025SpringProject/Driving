#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Range
from duckietown_msgs.msg import BoolStamped
import os
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse
import requests


class BarriersNode(Node):
    def __init__(self, bot_name=None):
        # Initialisation of the Node
        super().__init__('traffic_lights_node')
        if bot_name is None:
            self.get_logger().error('Bot name not specified')
        try:
            self.bot_name = os.getenv('VEHICLE_NAME')
        except Exception as e:
            self.get_logger().error('Bot name not specified')
            self.bot_name = bot_name
        self.ToF_topic = f'/{bot_name}/range'
        self.emergency_stop_topic = f'/{bot_name}/emergency_stop'
        self.stop_tof = False

        try:
            self.tof_subscription = self.create_subscription(
                Range,
                self.ToF_topic,
                self.tof_callback,
                1)
        except Exception as e:
            self.get_logger().error(f'Error while subscribing to ToF: {str(e)}')

        try:
            self.emergency_publisher = self.create_publisher(
                BoolStamped,
                self.emergency_stop_topic,
                1)
        except Exception as e:
            self.get_logger().error(f'Error while publishing to emergency: {str(e)}')

        # Logs about node start up
        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Subscribed to {self.ToF_topic}')

    def tof_callback(self, tof_msg):
        try:
            ran = tof_msg.range
            # self.get_logger().info(f'Range: {ran}')
            if ran < 0.1:
                emergency_stop = BoolStamped()
                self.stop_tof = True
                emergency_stop.data = True
                self.emergency_publisher.publish(emergency_stop)
                self.get_logger().info(f'Emergency stop published')
            else:
                self.stop_tof = False
                emergency_stop = BoolStamped()
                emergency_stop.data = False
                self.emergency_publisher.publish(emergency_stop)
        except Exception as e:
            self.get_logger().error(f'Error while working with tof: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    image_processor = BarriersNode(bot_name="duckie04")
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
