#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import LED

from gpiozero import Device
from gpiozero.pins.pigpio import PiGPIOFactory


# Force pigpio backend (prevents GPIO busy errors)
Device.pin_factory = PiGPIOFactory()


class LEDSubscriber(Node):
    def __init__(self):
        super().__init__('led_subscriber')

        # Define LEDs with GPIO pins
        self.leds = {
            'red': LED(14),
            'green': LED(12),
            'blue': LED(26),
            'yellow': LED(20)
        }

        # Subscriber for LED control messages
        self.subscription = self.create_subscription(
            String,
            'led_control',
            self.listener_callback,
            10
        )

        self.get_logger().info("LED subscriber started")

    def listener_callback(self, msg):
        led_name = msg.data.lower()

        if led_name in self.leds:
            self.leds[led_name].toggle()
            self.get_logger().info(f'Toggled LED: {led_name}')
        else:
            self.get_logger().warn(f'Unknown LED command: {led_name}')


def main(args=None):
    rclpy.init(args=args)
    node = LEDSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        
    
