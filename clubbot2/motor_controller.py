#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
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

        # Read parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_max_rpm = self.get_parameter('left_max_rpm').value
        self.right_max_rpm = self.get_parameter('right_max_rpm').value

        self.left_forward = self.get_parameter('left_forward_pin').value
        self.left_backward = self.get_parameter('left_backward_pin').value
        self.right_forward = self.get_parameter('right_forward_pin').value
        self.right_backward = self.get_parameter('right_backward_pin').value

        self.get_logger().info("Motor controller parameters loaded successfully.")

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_forward, GPIO.OUT)
        GPIO.setup(self.left_backward, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT)
        GPIO.setup(self.right_backward, GPIO.OUT)

        # PWM setup (1 kHz)
        self.lf_pwm = GPIO.PWM(self.left_forward, 1000)
        self.lb_pwm = GPIO.PWM(self.left_backward, 1000)
        self.rf_pwm = GPIO.PWM(self.right_forward, 1000)
        self.rb_pwm = GPIO.PWM(self.right_backward, 1000)

        self.lf_pwm.start(0)
        self.lb_pwm.start(0)
        self.rf_pwm.start(0)
        self.rb_pwm.start(0)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("Motor controller ready and listening to /cmd_vel")

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive equations
        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        # Convert linear velocity → RPM
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        # Clamp RPM
        rpm_l = max(min(rpm_l, self.left_max_rpm), -self.left_max_rpm)
        rpm_r = max(min(rpm_r, self.right_max_rpm), -self.right_max_rpm)

        # Convert RPM → PWM duty cycle (0–100)
        duty_l = abs(rpm_l) / self.left_max_rpm * 100.0
        duty_r = abs(rpm_r) / self.right_max_rpm * 100.0

        # Drive left motor
        if rpm_l >= 0:
            self.lf_pwm.ChangeDutyCycle(duty_l)
            self.lb_pwm.ChangeDutyCycle(0)
        else:
            self.lf_pwm.ChangeDutyCycle(0)
            self.lb_pwm.ChangeDutyCycle(duty_l)

        # Drive right motor
        if rpm_r >= 0:
            self.rf_pwm.ChangeDutyCycle(duty_r)
            self.rb_pwm.ChangeDutyCycle(0)
        else:
            self.rf_pwm.ChangeDutyCycle(0)
            self.rb_pwm.ChangeDutyCycle(duty_r)

    def destroy_node(self):
        # Stop PWM and clean up GPIO
        self.lf_pwm.stop()
        self.lb_pwm.stop()
        self.rf_pwm.stop()
        self.rb_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
        rclpy.init(args=args)
        node = MotorController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
