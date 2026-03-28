#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ButtonToLEDbridge(Node):
    def __init__(self):
        super().__init__('button_to_led_bridge')

        # Subscribe to button presses
        self.subscription = self.create_subscription(
            String,
            'button_press',
            self.button_callback,
            10
        )

        # Publisher to LED control
        self.led_pub = self.create_publisher(String, 'led_control', 10)

        # Map button names → LED names
        self.map = {
            'green': 'green',
            'blue': 'blue',
            'yellow': 'yellow'
        }

        self.get_logger().info("Button to LED bridge started")

    def button_callback(self, msg):
        button = msg.data.lower()

        if button in self.map:
            led = self.map[button]
            out = String()
            out.data = led
            self.led_pub.publish(out)
            self.get_logger().info(f"Button '{button}' to LED '{led}'")
        else:
            self.get_logger().warn(f"Unknown button: {button}")


def main(args=None):
    rclpy.init(args=args)
    node = ButtonToLEDbridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
