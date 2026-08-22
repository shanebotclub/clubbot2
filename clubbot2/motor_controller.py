#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters (ROS2 will load them from RobotParams.yaml)
        self.declare_parameter('wheel_diameter')
        self.declare_parameter('wheel_base')
        self.declare_parameter('left_max_rpm')
        self.declare_parameter('right_max_rpm')
        self.declare_parameter('left_forward_pin')
        self.declare_parameter('left_backward_pin')
        self.declare_parameter('right_forward_pin')
        self.declare_parameter('right_backward_pin')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        # Read parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_max_rpm = self.get_parameter('left_max_rpm').value
        self.right_max_rpm = self.get_parameter('right_max_rpm').value

        self.left_forward = self.get_parameter('left_forward_pin').value
        self.left_backward = self.get_parameter('left_backward_pin').value
        self.right_forward = self.get_parameter('right_forward_pin').value
        self.right_backward = self.get_parameter('right_backward_pin').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.get_logger().info("Motor controller parameters loaded successfully.")

        # gpiozero Motor setup
        self.left_motor = Motor(forward=self.left_forward, backward=self.left_backward)
        self.right_motor = Motor(forward=self.right_forward, backward=self.right_backward)
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Motor controller ready and listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        # Differential drive equations
        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        # Convert linear velocity → RPM
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        # Clamp RPM
        rpm_l = max(min(rpm_l, self.left_max_rpm), -self.left_max_rpm)
        rpm_r = max(min(rpm_r, self.right_max_rpm), -self.right_max_rpm)

        # Convert RPM → Normalized motor speed (-1.0 to 1.0)
        motor_val_l = rpm_l / self.left_max_rpm
        motor_val_r = rpm_r / self.right_max_rpm

        # Drive motors
        self.left_motor.value = motor_val_l
        self.right_motor.value = motor_val_r
    def destroy_node(self):
        # Stop and close motor devices
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()
        super().destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = MotorController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
