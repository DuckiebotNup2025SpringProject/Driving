#!/usr/bin/python3
import os
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import String


class TurnNode(Node):
    def __init__(self):
        super().__init__('turn_node')
        bot = os.getenv('VEHICLE_NAME', 'duckie02')
        self.pub = self.create_publisher(WheelsCmdStamped, f'/{bot}/wheels_cmd', 10)
        self.create_subscription(CompressedImage, f'/{bot}/image/compressed', self.img_cb, 10)
        self.create_subscription(WheelEncoderStamped, f'/{bot}/tick', self.enc_cb, 10)
        self.bridge = CvBridge()
        self.master_commands_topic = f'/{bot}/master_commands'
        self.master_callbacks_topic = f'/{bot}/master_callbacks'
        self.master_cmd_sub = self.create_subscription(
            String,
            self.master_commands_topic,
            self.master_cmd_cb,
            10
        )
        self.master_cb_pub = self.create_publisher(
            String,
            self.master_callbacks_topic,
            10
        )

        self.master_active = False
        self.master_last_cmd = ''
        self.dx = 0.0
        self.prev_dx = None
        self.front = False
        self.red_frac = 0.0
        self.path_m = 0.0
        self.ll = None
        self.rl = None
        self.ls = 0.0
        self.rs = 0.0
        self.state = 'STRAIGHT'
        self.stop_count = 0

        self.last_path_m = 0.0
        self.stuck_cnt = 0
        self.piv_scale = 1.0

        self.str_start = 0.0
        self.turn_start = None
        self.align_start = 0.0
        self.fwd_start = 0.0

        self.forward_timer = self.create_timer(0.1, self.forward_control_cb)
        self.left_timer = self.create_timer(0.30, self.left_control_cb)
        self.right_timer = self.create_timer(0.30, self.right_control_cb)

        self.forward_timer.cancel()
        self.left_timer.cancel()
        self.right_timer.cancel()

        self.active_mode = None

        self.get_logger().info('Initializing node')
        self.get_logger().info(f'Topics {self.master_commands_topic} {self.master_callbacks_topic}')

    def master_cmd_cb(self, msg: String):
        self.get_logger().info(f'Received command: {msg.data}')
        if msg.data not in ['TURN0', 'TURN90', 'TURN270']:
            return

        self.master_last_cmd = msg.data
        self.master_active = True

        turn_type = None
        if msg.data == 'TURN0':
            turn_type = 'forward'
        elif msg.data == 'TURN90':
            turn_type = 'right'
        elif msg.data == 'TURN270':
            turn_type = 'left'

        self.state = 'STRAIGHT'
        self.path_m = 0.0
        self.front = False
        self.stop_count = 0

        if self.active_mode is not None:
            if self.active_mode == 'forward':
                self.forward_timer.cancel()
            elif self.active_mode == 'left':
                self.left_timer.cancel()
            elif self.active_mode == 'right':
                self.right_timer.cancel()

        if turn_type == 'forward':
            self.active_mode = 'forward'
            self.forward_timer = self.create_timer(0.1, self.forward_control_cb)
        elif turn_type == 'left':
            self.active_mode = 'left'
            self.left_timer = self.create_timer(0.30, self.left_control_cb)
        elif turn_type == 'right':
            self.active_mode = 'right'
            self.str_start = 0.0
            self.right_timer = self.create_timer(0.30, self.right_control_cb)

    def enc_cb(self, msg: WheelEncoderStamped):
        if self.active_mode == 'forward':
            self.forward_enc_cb(msg)
        elif self.active_mode == 'left' or self.active_mode == 'right':
            self.line_guided_enc_cb(msg)

    def forward_enc_cb(self, msg: WheelEncoderStamped):
        d = msg.data / msg.resolution * (math.pi * 6.6 / 100)
        if self.ll is None:
            self.ll, self.ls = msg.data, d
            return
        if self.rl is None:
            if abs(msg.data - self.ll) > 10:
                self.rl, self.rs = msg.data, d
            else:
                self.ll, self.ls = msg.data, d
            return
        if abs(msg.data - self.ll) <= abs(msg.data - self.rl):
            delta = abs(d - self.ls)
            self.ll, self.ls = msg.data, d
        else:
            delta = abs(d - self.rs)
            self.rl, self.rs = msg.data, d
        self.path_m += delta
        self.get_logger().info(f'path_m={self.path_m:.3f}')

    def line_guided_enc_cb(self, msg: WheelEncoderStamped):
        d = msg.data / msg.resolution * (math.pi * 6.6 / 100)
        if self.ll is None:
            self.ll, self.ls = msg.data, d
            return
        if self.rl is None:
            if abs(msg.data - self.ll) > 10:
                self.rl, self.rs = msg.data, d
            else:
                self.ll, self.ls = msg.data, d
            return
        if abs(msg.data - self.ll) <= abs(msg.data - self.rl):
            self.path_m += abs(d - self.ls)
            self.ll, self.ls = msg.data, d
        else:
            self.path_m += abs(d - self.rs)
            self.rl, self.rs = msg.data, d

    def img_cb(self, msg: CompressedImage):
        if self.active_mode == 'left':
            self.left_img_cb(msg)
        elif self.active_mode == 'right':
            self.right_img_cb(msg)

    def left_img_cb(self, msg: CompressedImage):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None: return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
        edges = cv2.Canny(mask, 60, 180)
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180, 40, 30, 5)
        if lines is None: return
        h, w = mask.shape
        left, top = w, h
        for x1, y1, x2, y2 in lines[:, 0]:
            ang = abs(math.atan2(y2 - y1, x2 - x1))
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if ang > math.pi / 6:
                left = min(left, cx)
            else:
                top = min(top, cy)
        self.prev_dx, self.dx = self.dx, (left / w) - 0.23
        self.front |= (top / h) < 0.20

    def right_img_cb(self, msg: CompressedImage):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError:
            return
        if img is None or img.size == 0:
            return
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255)) | cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
        self.red_frac = np.count_nonzero(mask) / mask.size
        edges = cv2.Canny(mask, 60, 180)
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180, 40, 30, 5)
        if lines is None:
            return
        h, w = mask.shape
        right, top = 0, h
        for x1, y1, x2, y2 in lines[:, 0]:
            ang = abs(math.atan2(y2 - y1, x2 - x1))
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            if ang > math.pi / 6:
                right = max(right, cx)
            else:
                top = min(top, cy)
        self.dx = ((w - right) / w) - 0.23
        self.front |= (top / h) < 0.20

    def forward_control_cb(self):
        if not self.master_active:
            return
        self.publish(0.0, 0.0)
        time.sleep(0.01)
        if self.path_m >= 0.50:
            ack = String()
            ack.data = "SUCCESS"
            self.master_cb_pub.publish(ack)
            self.master_active = False
            self.get_logger().info(f'stop at {self.path_m:.3f}m')
            self.publish(0.0, 0.0)
            self.forward_timer.cancel()
            self.active_mode = None
            return
        vl = 0.20 - 0.05
        vr = 0.20 + 0.05
        self.get_logger().info(f'curve left vl={vl:.2f} vr={vr:.2f}')
        self.publish(vl, vr)

    def left_control_cb(self):
        if not self.master_active:
            return
        self.publish(0.0, 0.0)
        time.sleep(0.1)
        progressed = self.path_m - self.last_path_m
        self.last_path_m = self.path_m
        if progressed < 0.005:
            self.stuck_cnt += 1
        else:
            self.stuck_cnt = 0
            if self.piv_scale > 1.0:
                self.piv_scale = max(1.0, self.piv_scale - 0.02)
        if self.stuck_cnt >= 5 and self.piv_scale < (0.18 / 0.14):
            self.piv_scale = min(self.piv_scale + 0.02 / 0.14,
                                 0.18 / 0.14)
            self.stuck_cnt = 0
            self.get_logger().info(f'Pivot scale increased to {self.piv_scale:.2f}')
        v_piv = 0.14 * self.piv_scale
        V_PIV_L = (-v_piv, v_piv)
        V_PIV_R = (v_piv, -v_piv)
        self.get_logger().info(f'State: {self.state}, path: {self.path_m:.2f}, dx: {self.dx:.3f}')

        if self.state == 'STRAIGHT':
            self.publish(0.28, 0.28)
            if self.front or self.path_m >= 0.26:
                self.front = False
                self.state = 'TURN'
                self.get_logger().info('Transition to TURN')
                return

        elif self.state == 'TURN':
            if self.front:
                self.front = False
                self.state = 'ALIGN'
                self.align_start = self.path_m
                self.get_logger().info('Transition to ALIGN')
                return
            p = self.path_m / 1.05
            vl, vr = ((0.23, 0.60) if p < 0.40 else
                      (0.17, 0.65) if p < 0.92 else V_PIV_L)
            if p < 0.92:
                steer = max(-0.05, min(0.05, -0.65 * self.dx))
                vl += steer
                vr -= steer
            self.publish(vl, vr)
            if p >= 1.0:
                self.state = 'ALIGN'
                self.align_start = self.path_m
                self.get_logger().info('Transition to ALIGN')
                return

        elif self.state == 'ALIGN':
            if self.dx < -0.06:
                self.publish(*V_PIV_R)
                return
            steer = max(-0.05, min(0.05, -0.9 * self.dx))
            self.publish(0.28 + steer, 0.28 - steer)
            if abs(self.dx) < 0.02 or self.path_m - self.align_start >= 0.10:
                self.state = 'FWD'
                self.fwd_start = self.path_m
                self.get_logger().info('Transition to FWD')
                return

        elif self.state == 'FWD':
            self.publish(0.28, 0.28)
            if self.front or self.path_m - self.fwd_start >= 0.20:
                self.front = False
                self.state = 'STOP'
                self.get_logger().info('Transition to STOP')
                return

        elif self.state == 'STOP':
            self.publish(0.0, 0.0)
            self.stop_count += 1
            if self.stop_count >= 10:
                ack = String()
                ack.data = "SUCCESS"
                self.master_cb_pub.publish(ack)
                self.master_active = False
                self.publish(0.0, 0.0)
                self.left_timer.cancel()
                self.active_mode = None

    def right_control_cb(self):
        if not self.master_active:
            return
        self.publish(0.0, 0.0)
        time.sleep(0.1)
        if self.state == 'STRAIGHT':
            self.publish(0.28, 0.28)
            if self.front or self.red_frac > 0.01 or self.path_m - self.str_start >= 0.02:
                self.front = False
                self.state = 'TURN'
                self.turn_start = self.path_m
                return
        elif self.state == 'TURN':
            d = self.path_m - self.turn_start
            if d < 0.20:
                self.publish(0.28, 0.28)
                return
            if self.red_frac > 0.01:
                self.publish(0.20, -0.20)
                return
            self.publish(0.20, -0.20)
            if d > 0.14 * math.pi / 4:
                self.state = 'ALIGN'
                self.align_start = self.path_m
                return
        elif self.state == 'ALIGN':
            steer = max(-0.05, min(0.05, 0.9 * self.dx))
            self.publish(0.28 - steer, 0.28 + steer)
            if abs(self.dx) < 0.02 or self.path_m - self.align_start >= 0.10:
                self.state = 'FWD'
                self.fwd_start = self.path_m
                return
        elif self.state == 'FWD':
            self.publish(0.28, 0.28)
            if self.path_m - self.fwd_start >= 0.15:
                self.state = 'STOP'
                return
        elif self.state == 'STOP':
            self.publish(0.0, 0.0)
            self.stop_count += 1
            if self.stop_count >= 10:
                ack = String()
                ack.data = "SUCCESS"
                self.master_cb_pub.publish(ack)
                self.master_active = False
                self.publish(0.0, 0.0)
                self.right_timer.cancel()
                self.active_mode = None

    @staticmethod
    def clamp(v, l):
        return max(-l, min(l, v))

    def publish(self, vl, vr):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = vl
        msg.vel_right = vr
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TurnNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.publish(0.0, 0.0)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()