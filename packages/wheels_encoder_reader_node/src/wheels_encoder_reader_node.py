#!/usr/bin/env python3

import os
from rclpy.node import Node
import rclpy
import math
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(Node):
    def __init__(self, bot_name=None):
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception as e:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        super().__init__('turn_node')
        self._ticks_left = None
        self._ticks_right = None
        self.bot_name = bot_name
        self.wheel_encoder_topic = f'/{bot_name}/tick'
        self.count = 0
        self.iteration = 0
        self.timer_period = 1
        self.left_start_distance = -1
        self.right_start_distance = -1
        self.timer = self.create_timer(self.timer_period, self.publish_ticks)

        self.wheel_encoder_publisher = self.create_subscription(
            WheelEncoderStamped,
            self.wheel_encoder_topic,
            self.callback_left,
            10)


        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Publishing to {self.wheel_encoder_topic}')

    def callback_left(self, data):
        if self.iteration%2 == 0:
            if self.left_start_distance == -1:
                self.left_start_distance = data.data/data.resolution*(math.pi*6.6)
            self.get_logger().info(f"Left encoder resolution: {data.resolution}")
            self.get_logger().info(f"Left encoder ticks: {data.data}")
            self.get_logger().info(f"Left encoder distance: {(data.data/data.resolution)*(math.pi*6.6)-self.left_start_distance}")
            self._ticks_left = data.data

        else:
            if self.right_start_distance == -1:
                self.right_start_distance = data.data/data.resolution*(math.pi*6.6)
            self.get_logger().info(f"Right encoder resolution: {data.resolution}")
            self.get_logger().info(f"Right encoder ticks: {data.data}")
            self.get_logger().info(f"Right encoder distance: {(data.data/data.resolution)*(math.pi*6.6)-self.right_start_distance}")
            self._ticks_left = data.data
        self.iteration+=1

    def publish_ticks(self):
        if self._ticks_left is not None and self._ticks_right is not None:
            msg = f"[{self.bot_name}] Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
            self.get_logger().info(msg)
        else:
            self.get_logger().warn("Waiting for both encoders to publish ticks...")

def main(args=None):
    rclpy.init(args=args)
    image_processor = WheelEncoderReaderNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()