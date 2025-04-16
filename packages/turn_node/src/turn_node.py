#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

class TurnStopAfterDistanceNode(Node):
    def __init__(self):
        vehicle_name = "duckie02"
        super().__init__('turn_stop_after_distance_node')
        self.vehicle_name = vehicle_name

        self.cmd_topic = f'/{vehicle_name}/wheels_cmd'
        self.cmd_pub = self.create_publisher(WheelsCmdStamped, self.cmd_topic, 10)

        self.encoder_topic = f'/{vehicle_name}/tick'
        self.create_subscription(WheelEncoderStamped, self.encoder_topic, self.encoder_callback, 10)

        self.get_logger().info(f"TurnStopAfterDistanceNode initialized for {vehicle_name}")
        self.get_logger().info(f"Using encoder topic: {self.encoder_topic}")
        self.get_logger().info(f"Publishing wheels commands to: {self.cmd_topic}")

        self.initial_drive_duration = 5.0
        self.target_total_distance = 1.0
        self.overshoot = 0.02

        self.wheel_circumference_factor = (math.pi * 6.6) / 100

        self.left_start = None
        self.right_start = None
        self.left_distance = 0.0
        self.right_distance = 0.0

        self.left_last_raw = None
        self.right_last_raw = None
        self.raw_threshold = 10  # порог в тиках

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.start_time = self.get_clock().now()

    def encoder_callback(self, msg: WheelEncoderStamped):
        self.get_logger().info(f"Raw encoder message: data = {msg.data}, resolution = {msg.resolution}")
        current_distance = (msg.data / msg.resolution) * self.wheel_circumference_factor
        raw = msg.data

        if self.left_last_raw is None and self.right_last_raw is None:
            self.left_last_raw = raw
            self.left_start = current_distance
            self.left_distance = 0.0
            self.get_logger().info("Assigned first message to LEFT encoder.")
        elif self.left_last_raw is not None and self.right_last_raw is None:
            if abs(raw - self.left_last_raw) > self.raw_threshold:
                self.right_last_raw = raw
                self.right_start = current_distance
                self.right_distance = 0.0
                self.get_logger().info("Assigned message to RIGHT encoder (initial).")
            else:
                self.left_distance = current_distance - self.left_start
                self.left_last_raw = raw
                self.get_logger().info("Updated LEFT encoder.")
        else:
            diff_left = abs(raw - self.left_last_raw)
            diff_right = abs(raw - self.right_last_raw)
            if diff_left <= diff_right:
                self.left_distance = current_distance - self.left_start
                self.left_last_raw = raw
                self.get_logger().info("Updated LEFT encoder.")
            else:
                self.right_distance = current_distance - self.right_start
                self.right_last_raw = raw
                self.get_logger().info("Updated RIGHT encoder.")

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        msg = WheelsCmdStamped()
        msg.header.stamp = now.to_msg()

        if elapsed < self.initial_drive_duration:
            msg.vel_left = 0.1
            msg.vel_right = 0.3
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Initial drive phase ({elapsed:.1f}s): Driving left at constant speed.")
            return

        total_distance = abs(self.left_distance) + self.right_distance
        self.get_logger().info(f"Elapsed time: {elapsed:.1f}s, Total distance traveled: {total_distance:.3f} m")

        if total_distance < (self.target_total_distance - self.overshoot):
            msg.vel_left = 0.1
            msg.vel_right = 0.3
            self.cmd_pub.publish(msg)
            self.get_logger().info("Feedback phase: Continuing turn at full speed...")
        elif total_distance < self.target_total_distance:
            msg.vel_left = -0.2
            msg.vel_right = 0.2
            self.cmd_pub.publish(msg)
            self.get_logger().info("Final aggressive phase: Executing sharp pivot...")
        else:
            msg.vel_left = 0.0
            msg.vel_right = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info("Target total distance reached. Stopping robot.")
            self.timer.cancel()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TurnStopAfterDistanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
