#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Button

from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory


# Force pigpio backend (prevents GPIO busy errors)
Device.pin_factory = PiGPIOFactory()
#check git
class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')

        # Publisher for button presses
        self.publisher_ = self.create_publisher(String, 'button_press', 10)

        # Define buttons with GPIO pins
        self.buttons = {
            'green': Button(6),
            'blue': Button(5),
            'yellow': Button(15)
        }

        # Attach callbacks
        for name, button in self.buttons.items():
            button.when_pressed = lambda n=name: self.button_callback(n)

        # use get_logger() to print info message when the node starts
        self.get_logger().info("Button publisher started")

    def button_callback(self, name):
        msg = String()
        msg.data = name
        self.publisher_.publish(msg)
        # use get_logger() to print info message when a button is pressed as a f string
        self.get_logger().info(f'Button pressed: {name}')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




