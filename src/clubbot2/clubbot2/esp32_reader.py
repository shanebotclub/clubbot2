import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from clubbot_interfaces.msg import BumperStates

import serial
import time


class Esp32Reader(Node):
    def __init__(self):
        super().__init__('esp32_reader')

        # Adjust if needed
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # Publishers
        self.bump_pub = self.create_publisher(BumperStates, 'bumpers', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to poll serial
        self.timer = self.create_timer(0.01, self.read_serial)

        # For velocity calculation
        self.last_time = time.time()
        self.last_left = 0
        self.last_right = 0

        # Bumper order must match ESP32
        self.bumper_order = ['lf', 'mf', 'rf', 'lb', 'mb', 'rb']

        self.get_logger().info("ESP32 PCNT reader started")


    # -----------------------------
    # SERIAL READER
    # -----------------------------
    def read_serial(self):
        if self.serial.in_waiting <= 0:
            return

        line = self.serial.readline().decode(errors='ignore').strip()
        if not line:
            return

        parts = line.split()
        msg_type = parts[0]

        if msg_type == "BUMP":
            self.handle_bump(parts[1:])
        elif msg_type == "ENC":
            self.handle_enc(parts[1:])
        else:
            self.get_logger().debug(f"Unknown msg: {line}")


    # -----------------------------
    # BUMPER HANDLER
    # -----------------------------
    def handle_bump(self, fields):
        values = {}
        for f in fields:
            if '=' not in f:
                continue
            name, val = f.split('=', 1)
            values[name] = (val == '1')

        msg = BumperStates()
        try:
            msg.lf = values['lf']
            msg.mf = values['mf']
            msg.rf = values['rf']
            msg.lb = values['lb']
            msg.mb = values['mb']
            msg.rb = values['rb']
        except KeyError as e:
            self.get_logger().warn(f"Missing bumper field: {e}")
            return

        self.bump_pub.publish(msg)


    # -----------------------------
    # ENCODER HANDLER (PCNT)
    # -----------------------------
    def handle_enc(self, fields):
        # fields: ["left=12345", "right=12098"]
        values = {}
        for f in fields:
            if '=' not in f:
                continue
            name, val = f.split('=', 1)
            values[name] = int(val)

        left = values.get('left')
        right = values.get('right')

        if left is None or right is None:
            self.get_logger().warn("Missing encoder fields")
            return

        # Compute velocity
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        left_vel = (left - self.last_left) / dt
        right_vel = (right - self.last_right) / dt

        self.last_left = left
        self.last_right = right

        # Publish JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel', 'right_wheel']
        msg.position = [float(left), float(right)]
        msg.velocity = [float(left_vel), float(right_vel)]
        msg.effort = []

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
