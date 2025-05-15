#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
import numpy as np
from scipy.interpolate import CubicSpline
from numpy import cos, sin

from duckietown_msgs.msg import WheelsCmdStamped

#//////////////////////////////////////////////////////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////////////
#//////////////////  This is Autonomous Driving  //////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////////////
#//////////////////////////////////////////////////////////////////////////////////////


# TODO: Import this constants from ROS config
DEBUG = True                # Debug flag # TODO: Change it in Production
FRAMERATE = 28              # Create a reading from node if there exit a node, that is outputting it to batter performance.
USE_MAX_SPEED = False       # This flag will mean, that speed of one of the motors will be always 1.
MOTOR_PUB_RATE = 100        # How many times should be send motor coefficients per second

X_BASE, Y_BASE = -30.0, 0.0   # Base coordinates for the robot (middle of the chassis) TODO: Calculate and change it in Production
BASE_WIDTH = 100            # Base width of the robot TODO: Calculate and change it in Production

# Vars for processing the drive function
LOOKAHEAD = 100             # How many poits
MAX_SPEED = 100               # TODO: This is testing value. Change it in Production
SPEED_KOEF = 0.3            # Relative value to motor speed. For batter performance may be controlled through nodes
ANGLE_KOEF = 0.7            # How strong will the difference in angles will be affect the rotation of the robot

class DrivingNode(Node):

    # Initiation of the node and creating the subscriptions and publishers
    def __init__(self):
        self.master_flag = False
        self.x = X_BASE
        self.y = Y_BASE
        self.theta = 0
        self.mask = None
        self.get_ahead_func = lambda x : (x + LOOKAHEAD, 0)
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
        self.points_topic = f'/{bot_name}/mask/debug/points'
        self.master_commands_topic = f'/{bot_name}/master_commands'
        self.master_callbacks_topic = f'/{bot_name}/master_callbacks'

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

        self.master_cmd = self.create_subscription(
            String,
            self.master_commands_topic,
            self.master_callback,
            1)

        self.master_feedback = self.create_publisher(
            String,
            self.master_callbacks_topic,
            1)

        if DEBUG:
            self.points_pub = self.create_publisher(
                Float32MultiArray,
                self.points_topic,
                1)

        # CV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Logs about node start up
        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Subscribed to {self.segmentation_topic}')
        self.get_logger().info(f'Publishing to {self.motor_topic}, {self.points_topic}')

        self.timer = self.create_timer(1 / MOTOR_PUB_RATE, self.send_motor_commands)

    def master_callback(self, msg):
        self.get_logger().info(f'Master command received: {msg.data}')
        if msg.data == 'STRAIGHT':
            self.get_logger().info('Starting the driving')
            self.master_flag = True

    # IDK why do i need this function
    def feedback_callback(self, feedback):
        self.get_logger().info('Feedback: {0}'.format(feedback.feedback.sequence))

    # This is recieve the mask and saving it localy in the right format
    def mask_callback(self, msg):
        if self.master_flag:
            self.get_logger().info('Mask was received')
            try:
                # Convert ROS Image message to OpenCV image
                # Assuming the mask is already a binary image
                self.mask = self.bridge.compressed_imgmsg_to_cv2(msg)

            except Exception as e:
                self.get_logger().error(f'Error processing mask: {str(e)}')

            self.start_moving()

    # Initiation of the
    def start_moving(self):

        self.get_logger().info('Moving started')
        points = self.points_calc(self.mask)
        self.get_logger().info('Points were calculated')
        func = self.vector_calc(points)

        self.timer.cancel()
        self.get_ahead_func = func
        self.x = X_BASE
        self.y = Y_BASE
        self.theta = 0
        self.timer = self.create_timer(1 / MOTOR_PUB_RATE, self.send_motor_commands)
        #TODO: Looks finished. Need Testing


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
        if self.master_flag:
            motor_msg = WheelsCmdStamped()
            motor_msg.vel_left = float(vel_left)
            motor_msg.vel_right = float(vel_right)
            self.get_logger().info('Sending wheels commands to ros !!!!')
            try:
                self.motor_publisher.publish(motor_msg)
                self.get_logger().debug(f'Published motor values: [{vel_left}, {vel_right}]')
            except Exception as e:
                self.get_logger().error(f'Error publishing motor values: {str(e)}')

    def vector_calc(self, points):

        if len(points) <= 4:
            return self.get_ahead_func

        # Sample points defining the curve
        points_x = []
        points_y = []

        for point in points:
            points_x.append(point[0])
            points_y.append(point[1])

        points_x = np.array(points_x)
        points_y = np.array(points_y)

        coeffs = np.polyfit(points_x, points_y, 2)
        poly_quadratic = np.poly1d(coeffs)

        def get_lookahead_point(x):
            closest_x = x + LOOKAHEAD
            return closest_x, poly_quadratic(closest_x)

        return get_lookahead_point

    def send_motor_commands(self):
        if self.master_flag:
            self.get_logger().info('Calculating motor values')
            vel_left, vel_right = self.calculate_motor_values()
            self.get_logger().info('Start of motor publishing')
            self.motor_pub(vel_left, vel_right)
            self.get_logger().info(f'Sending was successfully made  AA')


    # THis is function, that calculates the points of the middle of the right line of the road
    # TODO: This function should be optimized. It goes through all pixels and calculates mean of them, that is O((n * m)^2) in worth case
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
                if x_block == height - step and width * 0.4 <= y_block <= width * 0.6:
                    if len(np.argwhere(mask_block == 3)) > block * step * 0.5:
                        msg = String()
                        msg.data = 'SUCCESS'
                        self.motor_pub(0, 0)
                        self.master_flag = False
                        time.sleep(1)
                        self.master_feedback.publish(msg)
                        self.get_logger().info('Sent to master SUCCESS')
                        return []
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

        try:
            self.get_logger().info(f'Found {len(points)} points')
            if len(points) < 2:
                point_for_pub = []
                for point in points:
                    point_for_pub.append(float(point[0]))
                    point_for_pub.append(float(point[1]))

                msg = Float32MultiArray()
                msg.data = point_for_pub
                self.points_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing points: {str(e)}')

        return points

    def calculate_motor_values(self):
        dt = 1 / MOTOR_PUB_RATE

        lookahead_x, lookahead_y = self.get_ahead_func(self.x)

        # Calculate steering angle
        angle_to_target = np.arctan2(lookahead_y - self.y, lookahead_x - self.x)
        steering_angle = (angle_to_target - self.theta) * ANGLE_KOEF

        if abs(self.x) > 500:
            return 0, 0

        if abs(steering_angle) < 0.001:
            self.x += MAX_SPEED * SPEED_KOEF * cos(self.theta) * dt
            self.y += MAX_SPEED * SPEED_KOEF * sin(self.theta) * dt
            return SPEED_KOEF, SPEED_KOEF


        shoulder = MAX_SPEED * SPEED_KOEF / (steering_angle)

        # Move the robot
        self.x += shoulder * (sin(self.theta + steering_angle) - sin(self.theta))
        self.y -= shoulder * (cos(self.theta + steering_angle) - cos(self.theta))

        self.theta += steering_angle  # Adjust heading smoothly

        vel_left = SPEED_KOEF * ((shoulder + BASE_WIDTH / 2) / shoulder)
        vel_right = SPEED_KOEF * ((shoulder - BASE_WIDTH / 2) / shoulder)

        return vel_left, vel_right

def main(args=None):
    rclpy.init(args=args)
    image_processor = DrivingNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
