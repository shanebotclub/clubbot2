import rclpy
from rclpy.node import Node
from clubbot_interfaces.msg import BumperStates
from clubbot_interfaces.msg import EncoderCounts

import serial

class Esp32Reader(Node):
    def __init__(self):
        super().__init__('esp32_reader')

        # Adjust to your actual serial device
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # Publishers
        self.bump_pub = self.create_publisher(BumperStates, 'bumpers', 10)
        self.enc_pub  = self.create_publisher(EncoderCounts, 'encoders', 10)

        # Timer to listen for serial
        self.timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info('esp32_reader started')

        # Fixed bumper order (must match ESP32)
        self.bumper_order = ['lf', 'mf', 'rf', 'lb', 'mb', 'rb']

    def read_serial(self):
        if self.serial.in_waiting <= 0:
            return

        line = self.serial.readline().decode(errors='ignore').strip()
        if not line:
            return

        # Example: "BUMP lf=1 mf=0 rf=1 lb=0 mb=1 rb=0"
        parts = line.split()
        msg_type = parts[0]

        if msg_type == 'BUMP':
            self.handle_bump(parts[1:])
        
        elif msg_type == 'ENC':
            self.handle_enc(parts[1:])
        
        else:
            self.get_logger().debug(f'Unknown msg type: {msg_type}')

    # Bumper message handler
    def handle_bump(self, fields):
        # fields: ["lf=1", "mf=0", ...]
        values = {}
        for f in fields:
            if '=' not in f:
                continue
            name, val = f.split('=', 1)
            values[name] = (val == '1')

        # Build message in fixed order
        msg = BumperStates()
        try:
            msg.lf = values['lf']
            msg.mf = values['mf']
            msg.rf = values['rf']
            msg.lb = values['lb']
            msg.mb = values['mb']
            msg.rb = values['rb']
        except KeyError as e:
            self.get_logger().warn(f'Missing bumper field: {e}')
            return

        self.bump_pub.publish(msg)

    # Encoder message handler
    def handle_enc(self, fields):
        # Example fields:
        # ["left_forward=123", "left_backward=0", "right_forward=98", "right_backward=1"]

        values = {}
        for f in fields:
            if '=' not in f:
                continue
            name, val = f.split('=', 1)
            try:
                values[name] = int(val)
            except ValueError:
                self.get_logger().warn(f'Bad encoder value: {f}')
                return

        msg = EncoderCounts()
        try:
            msg.left_forward  = values['left_forward']
            msg.left_backward = values['left_backward']
            msg.right_forward = values['right_forward']
            msg.right_backward = values['right_backward']
        except KeyError as e:
            self.get_logger().warn(f'Missing encoder field: {e}')
            return

        self.enc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Reader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
