#!/usr/bin/env python3
import os
from calendar import weekday
from itertools import dropwhile
from re import search
from tarfile import CompressionError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from numpy import cos, arctan
import argparse

from duckietown.msg import WheelCmd

# TODO: Import this constants from ROS config
FRAMERATE = 20              # Create a reading from node if there exit a node, that is outputting it to batter performance.
USE_MAX_SPEED = False       # This flag will mean, that speed of one of the motors will be always 1.
SPEED_KOEF = 0.7            # Relative value to motor speed. For batter performance may be controlled through nodes
MAX_SPEED = 1               # TODO: This is testing value. Change it in Production
X_BASE, Y_BASE = 0.0, 0.0   # Base coordinates for the robot (middle of the chassis) TODO: Calculate and change it in Production
BASE_WIDTH = 100            # Base width of the robot TODO: Calculate and change it in Production


class DrivingNode(Node):
    def __init__(self, bot_name=None):
        self.x = X_BASE
        self.mask = None
        self.func = lambda x: 0
        self.dfunc = lambda x: 0
        # Initialisation of the Node
        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception as e:
            self.get_logger().error('Bot name not specified')
            bot_name = "duckiebot"
        super().__init__('driving_node')
        self.bot_name = bot_name
        self.motor_topic = f'/{bot_name}/wheels_cmd'
        self.segmentation_topic = f'/{bot_name}/mask'

        # Set up the image subscriber
        try:
            self.image_subscription = self.create_subscription(
                CompressedImage,
                self.segmentation_topic,
                self.mask_callback,
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

        self.timer = self.create_timer(0.01, self.send_motor_commands())

    def feedback_callback(self, feedback):
        self.get_logger().info('Feedback: {0}'.format(feedback.feedback.sequence))

    def mask_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            # Assuming the mask is already a binary image
            self.mask = self.bridge.imgmsg_to_cv2(msg)

            self.start_moving()

        except Exception as e:
            self.get_logger().error(f'Error processing mask: {str(e)}')

    def start_moving(self):

        points = self.points_calc(self.mask)
        self.func, self.dfunc = self.driving_fun_gen(points)
        # Publish motor control values
        self.timer.cancel()
        self.x = X_BASE
        self.timer = self.create_timer(0.01, self.send_motor_commands)

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

    def driving_fun_gen(self, points):

        # Extract x, y, and weights from the triples
        x = np.array([t[0] for t in triples])
        y = np.array([t[1] for t in triples])
        weights = np.array([t[2] for t in triples])

        # Fit a 4th degree polynomial using weighted least squares
        coeffs = np.polyfit(x, y, 4, w=weights)

        def func(x):
            return coeffs[0] * x ** 4 + coeffs[1] * x ** 3 + coeffs[2] * x ** 2 + coeffs[3] * x + coeffs[4]

        def dfunc(x):
            dx = 0.0001
            return (func(x + dx) - func(x)) / dx

        return func, dfunc

    def send_motor_commands(self):
        vel_left, vel_right = self.calculate_motor_values(self.x)
        self.motor_publisher(vel_left, vel_right)
        self.x += MAX_SPEED * SPEED_KOEF / FRAMERATE

    def points_calc(self, mask, step=20, block = 20):
        points = []

        # mask shape
        height, width = mask.shape[:2]

        # two initial points (for start and vector direction)
        points.append((X_BASE, Y_BASE, 100000))
        points.append((X_BASE + 0.001, Y_BASE, 100000))

        #search in the mask by lines with step = step
        iter = -1
        for x_block in range(height - step, 0, -step):

            yellow_center, white_left_center, white_right_center = (0, 0), (0, 0), (0, 0)
            y_flag, wl_flag, wr_flag = False, False, False

            #search in blocks with size (step x block)
            for y_block in range(0, width - 1, block):
                mask_block = mask[x_block:x_block + step, y_block:y_block + block] # cut out the block
                yellow = np.argwhere(mask_block == 1) # TODO: Change this to the actual color of yellow
                white = np.argwhere(mask_block == 2) # TODO: Change this to the actual color of white
                # if there are more than 30% of the block in the color, then we consider that the color is present
                if len(yellow) > block * step * 0.3:
                    y_flag = True
                    yellow_center = np.mean(yellow, axis=0)
                if len(white) > block * step * 0.3:
                    # TODO: Find better algorithm to find left border line
                    if y_flag or y_block + block / 2 > width / 2:
                        if not wr_flag: #for finding the nearest to the center (or yellow line) right border
                            wr_flag = True
                            white_right_center = np.mean(white, axis=0)
                    else: #for finding the nearest to the center (or yellow line) left border
                        wl_flag = True
                        white_left_center = np.mean(white, axis=0)

            #find id of the line where we are
            iter += 1
            if y_flag and (wl_flag or wr_flag): #when we found a yellow line and one of the white lines
                if wr_flag: #default search
                    x, y = ((yellow_center[0] + white_right_center[0]) / 2,
                            (yellow_center[1] + white_right_center[1]) / 2)
                else: #search based on left border and yellow line
                    x, y = ((yellow_center[0] * 3 - white_left_center[0]) / 2,
                            (yellow_center[1] * 3 - white_left_center[1]) / 2)
                points.append((x, y, 2 ** iter))
            elif wl_flag and wr_flag: #search based on two borders (not accurate at all. May be deleted for accuracy)
                self.get_logger().info('Calculating path based on the borders')
                x, y = ((white_left_center[0] * 3 + white_right_center[0]) / 4,
                        (white_left_center[1] * 3 + white_right_center[1]) / 4)
                points.append((x, y, 2 ** iter))
            else:
                self.get_logger().info(f'Not enough data in line to find center of road')
                continue

        return points

    def calculate_motor_values(self, x):
        dx = MAX_SPEED * SPEED_KOEF / FRAMERATE
        g0, g1 = self.dfunc(x), self.dfunc(x + dx)
        alfa = arctan(g1) - arctan(g0)
        shoulder = dx / cos((arctan(g1) + arctan(g0)) / 2) / alfa
        vel_left = SPEED_KOEF * ((shoulder + BASE_WIDTH / 2) / BASE_WIDTH)
        vel_right = SPEED_KOEF * ((shoulder - BASE_WIDTH / 2) / BASE_WIDTH)
        return vel_left, vel_right

def main(args=None):
    rclpy.init(args=args)
    image_processor = DrivingNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
