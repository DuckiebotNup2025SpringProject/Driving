#!/bin/python3
import os
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sympy.codegen.ast import continue_


class DrivingMode(Enum):
    MANUAL = 0
    AUTONOMOUS = 1


class Target(Enum):
    PAUSED = 0
    MOVING = 1
    TURNING = 2
    TRAFFIC_LIGHTING = 3


class MasterNode(Node):
    def __init__(self):
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
        self.apriltag_topic = f'/{bot_name}/apriltag'

        self.mode = DrivingMode.AUTONOMOUS
        self.target = Target.MOVING
        self.tasks_list = deque(["T270", "S", "T0", "S", "T90", "S"])
        self.turning_angle = ""
        self.next_node = ""
        self.last_command = "S"
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

        self.apriltag_subscription = self.create_subscription(
            String,
            self.apriltag_topic,
            self.parse_tag,
            10)

        self.master_publisher = self.create_publisher(
            String,
            self.master_commands_topic,
            10)

        self.action_timer = self.create_timer(0.3, lambda: self.action())

    def parse_tag(self, msg):
        try:
            tag = msg.data
            if msg is None:
                return
            self.last_apriltag = str(tag)
        except Exception:
            self.get_logger().error("Error parsing tag")

    def add_route(self, msg):
        # self.tasks_list = deque(msg.data.split(','))
        self.mode = DrivingMode.AUTONOMOUS

    def action(self):
        self.get_logger().error("Action started" + str(self.target))
        if self.mode == DrivingMode.AUTONOMOUS:
            if self.target == Target.MOVING:
                msg = String()
                msg.data = "STRAIGHT"
                self.master_publisher.publish(msg)
                self.get_logger().error("Message straight published")
                self.target = Target.PAUSED
            elif self.target == Target.TURNING:
                msg = String()
                msg.data = "TURN" + self.turning_angle
                self.master_publisher.publish(msg)
                self.target = Target.PAUSED
            elif self.target == Target.TRAFFIC_LIGHTING:
                msg = String()
                msg.data = "TRAFFIC"
                self.master_publisher.publish(msg)
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
        elif msg.data == "no_traffic_lights":
            pass
        elif msg.data[:7] == "Forward":
            data = msg.data
            data = data.split(', ')
            forward = bool(data[0][7:])
            left = bool(data[0][4:])
            right = bool(data[0][5:])
            if (self.last_command == "T0" and forward) or (self.last_command == "T90" and right) or (
                    self.last_command == "T270" and left):
                pass
            else:
                msg = String()
                msg.data = "TRAFFIC"
                self.master_publisher.publish(msg)
                return
            if self.tasks_list:
                self.evaluate_task(self.tasks_list.popleft())
            else:
                self.get_logger().info(f'Task is completed, turned to manual mode')
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
        elif task[:1] == "T":
            self.turning_angle = task[1:]
            self.target = Target.TURNING


def main(args=None):
    rclpy.init(args=args)
    image_processor = MasterNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
