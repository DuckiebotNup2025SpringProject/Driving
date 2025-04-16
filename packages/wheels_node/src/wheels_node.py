#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node

from duckietown_msgs.msg import WheelsCmdStamped

class WheelsNode(Node):
    def __init__(self, bot_name=None):
        # Initialisation of the Node
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception as e:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        super().__init__('turn_node')
        self.bot_name = bot_name
        self.wheels_topic = f'/{bot_name}/wheels_cmd_stamped'
        self.count = 0

        self.wheels_publisher = self.create_publisher(
            WheelsCmdStamped,
            self.wheels_topic,
            1)

        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Publishing to {self.wheels_topic}')

        self.timer = self.create_timer(5, self.turn_wheels)

    def turn_wheels(self):
        wheels_state_1 = WheelsCmdStamped()
        wheels_state_1.vel_left = 1.0
        wheels_state_1.vel_right = 0.0
        wheels_state_2 = WheelsCmdStamped()
        wheels_state_2.vel_left = 0.0
        wheels_state_2.vel_right = 1.0
        wheels_state_3 = WheelsCmdStamped()
        wheels_state_3.vel_left = 1.0
        wheels_state_3.vel_right = 1.0
        wheels_state_4 = WheelsCmdStamped()
        wheels_state_4.vel_left = 0.0
        wheels_state_4.vel_right = 0.0
        wheels_states = [wheels_state_1, wheels_state_2, wheels_state_3, wheels_state_4]
        try:
            self.wheels_publisher.publish(wheels_states[self.count%4])
            self.get_logger().info(f'Все норм, {self.count}')
        except Exception:
            pass
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    image_processor = WheelsNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
