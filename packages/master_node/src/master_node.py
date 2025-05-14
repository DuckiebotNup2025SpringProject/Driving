#!/bin/python3
import os

import apriltag
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from enum import Enum
from collections import deque


class DrivingMode(Enum):
    MANUAL = 0
    AUTONOMOUS = 1


class Target(Enum):
    PAUSED = 0
    MOVING = 1
    TURNING = 2
    TRAFFIC_LIGHTING = 3


class MasterNode(Node):
    def __init__(self, bot_name=None):
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        super().__init__('master_node')
        self.bot_name = bot_name
        self.master_commands_topic = f'/{bot_name}/master_commands'
        self.master_callbacks_topic = f'/{bot_name}/master_callbacks'
        self.route_topic = f'/{bot_name}/route'

        self.mode = DrivingMode.MANUAL
        self.target = Target.MOVING
        self.tasks_list = deque()
        self.turning_angle = ""
        self.next_node = ""
        self.last_command = ""
        self.last_apriltag = ""

        self.get_logger().info(f'Node initialized for {bot_name}')

        self.master_subscription = self.create_subscription(
            String,
            self.master_callbacks_topic,
            self.listen,
            10)

        self.route_subscription = self.create_subscription(
            String,
            self.route_topic,
            self.add_route,
            10)

        self.master_publisher = self.create_publisher(
            String,
            self.master_commands_topic,
            10)

        self.action_timer = self.create_timer(0.1, lambda: self.action())

    def add_route(self,msg):
        self.tasks_list = deque(msg.data.split(','))
        self.mode = DrivingMode.AUTONOMOUS

    def action(self):
        if self.mode == DrivingMode.AUTONOMOUS:
            if self.target == Target.MOVING:
                msg = String()
                msg.data = str("STRAIGHT")
                self.master_publisher.publish(msg)
                self.target = Target.PAUSED
            elif self.target == Target.TURNING:
                self.master_publisher.publish("TURN" + self.turning_angle)
                self.target = Target.PAUSED
            elif self.target == Target.TRAFFIC_LIGHTING:
                self.master_publisher.publish("TRAFFIC")
                self.target = Target.PAUSED
            elif self.target == Target.PAUSED:
                pass
        elif self.mode == DrivingMode.MANUAL:
            pass

    def listen(self, msg):
        if msg.data == "SUCCESS":
            if self.last_command == "S":
                pass
            elif self.last_command[0] == "S":
                if self.last_command[1:] != self.last_apriltag:
                    self.get_logger().info(f'Error, turned to manual mode')
                    self.mode = DrivingMode.MANUAL

            if self.tasks_list:
                self.evaluate_task(self.tasks_list.popleft())
            else:
                self.get_logger().info(f'Task is completed, turned to manual mode')
                self.mode = DrivingMode.MANUAL
        else:
            self.get_logger().info(f'Error, turned to manual mode')
            self.mode = DrivingMode.MANUAL

    def evaluate_task(self, task):
        self.last_command = task
        if task == "S":
            self.target = Target.MOVING
        elif task[0] == "S":
            self.next_node = task[1:]
            self.target = Target.MOVING
        elif task[:4] == "TURN":
            self.turning_angle = task[4:]
            self.target = Target.TURNING


def main(args=None):
    rclpy.init(args=args)
    image_processor = MasterNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
