#!/usr/bin/env python3
import os
from calendar import weekday
from itertools import dropwhile
from re import search
from tarfile import CompressionError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.interpolate import CubicSpline
from numpy import cos, arctan
import argparse

from duckietown_msgs.msg import WheelsCmdStamped

# TODO: Import this constants from ROS config
DEBUG = True                # Debug flag # TODO: Change it in Production
FRAMERATE = 20              # Create a reading from node if there exit a node, that is outputting it to batter performance.
USE_MAX_SPEED = False       # This flag will mean, that speed of one of the motors will be always 1.
SPEED_KOEF = 0.2            # Relative value to motor speed. For batter performance may be controlled through nodes
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
            bot_name = "example_robot"
        super().__init__('driving_node')
        self.bot_name = bot_name
        self.motor_topic = f'/{bot_name}/wheels_cmd'
        self.segmentation_topic = f'/{bot_name}/mask'
        self.motor_coef_topic = f'/{bot_name}/mask/path/line/debug'
        self.points_topic = f'/{bot_name}/mask/debug/points'

        # Set up the image subscriber
        self.image_subscription = self.create_subscription(
            CompressedImage,
            self.segmentation_topic,
            self.mask_callback,
            1)

        # Set up the motors publisher
        self.motor_publisher = self.create_publisher(
            WheelsCmdStamped,
            self.motor_topic,
            1)

        if DEBUG:
            self.path_publisher = self.create_publisher(
                Float32MultiArray,
                self.motor_coef_topic,
                1)

            self.points_pub = self.create_publisher(
                Float32MultiArray,
                self.points_topic,
                1)
        # CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Logs about node start up
        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Subscribed to {self.segmentation_topic}')
        self.get_logger().info(f'Publishing to {self.motor_topic}, {self.motor_coef_topic}, {self.points_topic}')

        self.timer = self.create_timer(0.01, self.send_motor_commands)

    def feedback_callback(self, feedback):
        self.get_logger().info('Feedback: {0}'.format(feedback.feedback.sequence))

    def mask_callback(self, msg):
        self.get_logger().info('Mask was received')
        try:
            # Convert ROS Image message to OpenCV image
            # Assuming the mask is already a binary image
            self.mask = self.bridge.compressed_imgmsg_to_cv2(msg)

        except Exception as e:
            self.get_logger().error(f'Error processing mask: {str(e)}')

        self.start_moving()

    def start_moving(self):

        self.get_logger().info('Moving started')
        points = self.points_calc(self.mask)
        self.get_logger().info('Points were calculated')
        self.func, self.dfunc = self.driving_fun_gen(points)
        self.get_logger().info('Driving functions generated')
        # Publish motor control values
        self.get_logger().info(f'Restarting timer')
        self.timer.cancel()
        self.x = X_BASE
        self.timer = self.create_timer(0.01, self.send_motor_commands)
        self.get_logger().info('Motor commands restarted')

    def motor_pub(self, vel_left, vel_right):
        if not (-1.0 < vel_left < 1.0):
            self.get_logger().error(f'Velocity left out of range: {vel_left}')
            if vel_left > 0:
                vel_left = 1.0
            else:
                vel_left = -1.0
            vel_left = max(-1.0, min(1.0, vel_left))
        if not (-1.0 < vel_right < 1.0):
            self.get_logger().error(f'Velocity right out of range: {vel_right}')
            if vel_right > 0:
                vel_right = 1.0
            else:
                vel_right = -1.0
            vel_right = max(-1.0, min(1.0, vel_right))
        motor_msg = WheelsCmdStamped()
        motor_msg.vel_left = float(vel_left)
        motor_msg.vel_right = float(vel_right)
        self.get_logger().info('Sending wheels commands to ros !!!!')
        try:
            self.motor_publisher.publish(motor_msg)
            self.get_logger().debug(f'Published motor values: [{vel_left}, {vel_right}]')
        except Exception as e:
            self.get_logger().error(f'Error publishing motor values: {str(e)}')

    def driving_fun_gen(self, triples):

        # Extract x, y, and weights from the triples
        l = []
        points_x = []
        points_y = []
        for i in range(len(triples)):
            points_x.append(triples[i][0])
            points_y.append(triples[i][1])
            l.append(triples[i][0])
            l.append(triples[i][1])
        msg = Float32MultiArray()
        msg.data = l
        self.points_pub.publish(msg)

        # Create a cubic spline interpolation of the points
        spline = CubicSpline(points_x, points_y)

        # Define the inner function that calculates y based on x
        def func(x):
            return spline(x)

        if DEBUG:
            arr = []
            for x in range(self.mask.shape[0]):
                y = func(x)
                if x < self.mask.shape[0] and abs(y) < self.mask.shape[1]:
                    arr.append(x)
                    arr.append(func(x))
            massage = Float32MultiArray()
            massage.data = arr
            self.path_publisher.publish(massage)
            self.get_logger().info('Path line was published')

        def dfunc(x):
            dx = 0.0001
            return (func(x + dx) - func(x)) / dx

        return func, dfunc

    def send_motor_commands(self):
        self.get_logger().info('Calculating motor values')
        vel_left, vel_right = self.calculate_motor_values(self.x)
        self.get_logger().info('Start of motor publishing')
        self.motor_pub(vel_left, vel_right)
        self.x += MAX_SPEED * SPEED_KOEF * 0.01
        self.get_logger().info(f'Sending was successfully made  AA')

    def points_calc(self, mask, step=10, block = 7):
        points = []

        # mask shape
        height, width = mask.shape[:2]

        # two initial points (for start and vector direction)
        points.append((X_BASE, Y_BASE, 1000000))
        points.append((X_BASE + 0.1, Y_BASE, 1000000))
        points.append((X_BASE + 2, Y_BASE, 100))

        #search in the mask by lines with step = step
        iter = -1
        for x_block in range(height - step, int(0.2 * height), -step):

            yellow_center, white_left_center, white_right_center = (np.array([0, 0]),
                                                                    np.array([0, 0]),
                                                                    np.array([0, 0]))
            y_flag, wl_flag, wr_flag = False, False, False

            yellow_line = []
            #search in blocks with size (step x block)
            for y_block in range(0, width - 1, block):
                mask_block = mask[x_block:x_block + step, y_block:y_block + block] # cut out the block
                yellow = np.argwhere(mask_block == 2)
                white = np.argwhere(mask_block == 1)
                # if there are more than 30% of the block in the color, then we consider that the color is present
                if len(yellow) > block * step * 0.3:
                    y_flag = True
                    yellow_line.append(np.mean(yellow, axis=0) +
                                     np.array([height - x_block, (y_block - width / 2)]))

                    #points.append((yellow_center[0], yellow_center[1], 0))
                if len(white) > block * step * 0.4:
                    if y_flag or y_block + block / 2 > width / 2:
                        if not wr_flag: #for finding the nearest to the center (or yellow line) right border
                            wr_flag = True
                            white_right_center = (np.mean(white, axis=0) +
                                                      np.array([height - x_block, (y_block - width / 2)]))
                            #points.append((white_right_center[0], white_right_center[1], 0))
                    else: #for finding the nearest to the center (or yellow line) left border
                        wl_flag = True
                        white_left_center = (np.mean(white, axis=0) +
                                             np.array([height - x_block, (y_block - width / 2)]))
                        #points.append((white_left_center[0], white_left_center[1], 0))

            yellow_center = np.mean(yellow_line, axis=0)
            #find id of the line where we are
            iter += 1
            if y_flag and (wl_flag or wr_flag): #when we found a yellow line and one of the white lines
                if wr_flag: #default search
                    #self.get_logger().info(f'Calculating path based on yellow and right border {yellow_center[1]} {white_right_center[1]}')
                    x, y = ((yellow_center[0] + white_right_center[0]) / 2,
                            (yellow_center[1] + white_right_center[1]) / 2)
                    #points.append((yellow_center[0], yellow_center[1], 0))
                    #points.append((white_right_center[0], white_right_center[1], 0))
                else: #search based on left border and yellow line
                    #self.get_logger().info('Calculating path based on yellow and left border')
                    x, y = ((yellow_center[0] * 3 - white_left_center[0]) / 2,
                            (yellow_center[1] * 3 - white_left_center[1]) / 2)
                    #points.append((yellow_center[0], yellow_center[1], 0))
                    #points.append((white_left_center[0], white_left_center[1], 0))
                points.append((x, y, 10))
            elif wl_flag and wr_flag: #search based on two borders (not accurate at all. May be deleted for accuracy)
                #self.get_logger().info('Calculating path based on the borders')
                x, y = ((white_left_center[0] + white_right_center[0] * 3) / 4,
                        (white_left_center[1] + white_right_center[1] * 3) / 4)
                #points.append((white_left_center[0], white_left_center[1], 0))
                #points.append((white_right_center[0], white_right_center[1], 0))
                points.append((x, y, 10))
            else:
                #self.get_logger().info(f'Not enough data in line to find center of road. Line {iter}')
                continue

        return points

    def calculate_motor_values(self, x):
        dx = MAX_SPEED * SPEED_KOEF / FRAMERATE
        g0, g1 = self.dfunc(x), self.dfunc(x + dx)
        alfa = arctan(g1) - arctan(g0)
        if cos((arctan(g1) + arctan(g0)) / 2) * alfa <= 0.0001:
            return SPEED_KOEF, SPEED_KOEF
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
