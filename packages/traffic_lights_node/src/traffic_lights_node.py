#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import String
from cv_bridge import CvBridge
import requests
import cv2
import numpy as np


class TrafficLightsNode(Node):
    WHITE = (250, 250, 250)
    BLACK = (0, 0, 0)

    @staticmethod
    def is_black(pix):
        return pix[0] == 0 and pix[1] == 0 and pix[2] == 0

    @staticmethod
    def crossroad_crop(img):
        size_y, size_x = img.shape[0], img.shape[1]
        img_copy = img[size_y // 10:size_y // 3, 0: size_x - size_x // 4]
        return img_copy

    @staticmethod
    def sum_pix(pix1):
        return int(pix1[0]) + int(pix1[1]) + int(pix1[2])

    def __init__(self, bot_name=None):
        # Initialisation of the Node
        super().__init__('traffic_lights_node')
        if bot_name is None:
            self.get_logger().error('Bot name not specified')

        try:
            bot_name = os.getenv('VEHICLE_NAME')
        except Exception:
            self.get_logger().error('Bot name not specified')
            bot_name = "example_robot"
        self.bot_name = bot_name
        self.bridge = CvBridge()
        self.image_topic = f'/{bot_name}/image/compressed'
        self.recognition_service_name = f"/{bot_name}/recognition"
        self.master_commands_topic = f'/{bot_name}/master_commands'
        self.master_callbacks_topic = f'/{bot_name}/master_callbacks'
        self.should_recognize = False

        try:
            self.image_subscription = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.image_callback,
                1)
        except Exception as e:
            self.get_logger().error(f'Error while subscribing to segmentation {e}')

        try:
            self.master_subscription = self.create_subscription(
                String,
                self.master_commands_topic,
                self.may_be_call,
                10
            )
        except Exception as e:
            self.get_logger().error(f'Error while subscribing to master {e}')

        # "no_traffic_lights" or "Forward-{bool}, Left-{bool}, Right-{bool}"
        try:
            self.master_publisher = self.create_publisher(
                String,
                self.master_callbacks_topic,
                10
            )
        except Exception as e:
            self.get_logger().error(f'Error while publishing master {e}')

        # Logs about node start up
        self.get_logger().info(f'Node initialized for {bot_name}')
        self.get_logger().info(f'Subscribed to {self.image_topic}')
        self.get_logger().info(f'Created service: {self.recognition_service_name}')

        self.png_buffer = None

    def image_callback(self, im_msg):
        try:
            self.png_buffer = self.bridge.compressed_imgmsg_to_cv2(im_msg)
        except Exception as e:
            self.get_logger().error(f'Error while converting image {e}')

    def may_be_call(self, msg):
        self.get_logger().info(f'Received {msg.data}')
        data = str(msg.data)
        if data == "TRAFFIC":
            res = self.recognize_req()
            self.master_publisher.publish(res)


    def recognize_req(self):
        self.get_logger().info(f'Received recognition request')
        ans = self.recognize_model()
        count = 1
        res = String()
        while (ans[0] == "again" or ans == "again") and count < 2:
            count += 1
            self.png_buffer = cv2.flip(self.png_buffer, 1)
            ans = self.recognize_model()
        if (ans[0] == "again" or ans == "again"):
            self.get_logger().info(f'No traffic lights detected')
            an = "no_traffic_lights"
            res.data = an
        else:
            an = String()
            forw = False
            left = False
            right = False
            if ans[0] == "g":
                forw = True
            if "l" in ans[1]:
                left = True
            if "r" in ans[1]:
                right = True
            an.data = f"Forward-{forw}, Left-{left}, Right-{right}"
            res.data = an
        return res

    def recognize_model(self):
        # img = self.crossroad_crop(self.png_buffer)
        try:
            img = self.png_buffer
            position = self.compare_guesses(img)
            self.get_logger().info(f'Traffic lights position: {position}')
            if position == "again":
                return "again"
            img = img[position[1]:position[3], position[0]:position[2]]
            self.get_logger().info(f'Image cropped to {img.shape}')
            res = self.recognize_signal_alter(img)
            return res
        except Exception as e:
            self.get_logger().error(f'Error while cropping/managing {e}')
            return "again"

    def compare_guesses(self, img):
        try:
            _, buffer = cv2.imencode(".png", img)
            im_bytes = buffer.tobytes()
            self.get_logger().info(f'Sending {len(img)} bytes')
            url = 'http://192.168.0.109:9595/recognize'
            files = {'file': (f"rec.png", im_bytes)}
            response = requests.post(url, files=files)
            res = response.json()
            if res["message"] == "again":
                return "again"
            coor = [int(i) for i in res["message"].split(", ")]

            '''for res in results:
                boxes = res.boxes.cpu().numpy()
                xyxy = boxes.xyxy
                position = int((xyxy[0][0] + xyxy[0][2]) / 2)'''
            return coor
        except Exception as e:
            self.get_logger().error(f'Error while recognizing image {e}')
            return "again"

    def recognize_signal_alter(self, img):
        try:
            img2 = img.copy()
            mean = img.mean()

            for y in range(img.shape[0]):
                for x in range(img.shape[1]):
                    if self.sum_pix(img[y, x]) / 3 > mean:
                        img2[y, x] = self.WHITE
                    else:
                        img2[y, x] = self.BLACK

            first_black = 0
            for y in range(img.shape[0]):
                for x in range(img.shape[1] // 2 - 3, img.shape[1] // 2 + 3):
                    if self.is_black(img2[y, x]):
                        first_black = y
                        break
                if first_black != 0:
                    break

            left_corner = -1
            right_corner = -1
            posl = 0
            for x in range(img.shape[1]):
                if self.is_black(img2[first_black + 1, x]):
                    posl += 1
                elif posl > 2:
                    right_corner = x
                    left_corner = x - posl
                    break
            left_arrow_count_light = 0
            for y in range(first_black + 20, first_black + 45):
                for x in range(left_corner):
                    if self.sum_pix(img[y][x]) > 600:
                        left_arrow_count_light += 1
            right_arrow_count_light = 0
            for y in range(first_black + 20, first_black + 45):
                for x in range(right_corner, img.shape[1]):
                    if self.sum_pix(img[y][x]) > 600:
                        right_arrow_count_light += 1
            count_red = 0
            count_yellow = 0
            count_green = 0
            for y in range(first_black, first_black + 50):
                for x in range(left_corner + 2, right_corner - 2):
                    if self.sum_pix(img[y][x]) > 600 and first_black + 22 > y > first_black + 13:
                        count_yellow += 1
                    elif self.sum_pix(img[y][x]) > 600 and y <= first_black + 13:
                        count_red += 1
                    elif self.sum_pix(img[y][x]) > 600:
                        count_green += 1

            answ = ["again", ""]
            if count_yellow > count_red and count_yellow > count_green:
                answ[0] = "y"
            elif count_red > count_green and count_red > count_yellow:
                answ[0] = "r"
            elif count_green != 0:
                answ[0] = "g"

            if left_arrow_count_light >= 3:
                answ[1] += "l"

            if right_arrow_count_light >= 3:
                answ[1] += "r"
            self.get_logger().info(f"red: {count_red}, yellow: {count_yellow}, green: {count_green}")
            return answ
        except Exception as e:
            self.get_logger().error(f'Error while recognizing signal {e}')
            return "again"


def main(args=None):
    rclpy.init(args=args)
    image_processor = TrafficLightsNode()
    rclpy.spin(image_processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
