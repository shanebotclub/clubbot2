#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import math
import yaml

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Load robot params
        with open('/home/ubuntu/ros2_ws/src/clubbot/RobotParams.yaml', 'r') as f:
            params = yaml.safe_load(f)['/**']['ros__parameters']

        self.wheel_diameter = params['wheel_diameter']
        self.wheel_base = params['wheel_base']
        self.left_max_rpm = params['left_max_rpm']
        self.right_max_rpm = params['right_max_rpm']

        self.left_forward = params['left_forward_pin']
        self.left_backward = params['left_backward_pin']
        self.right_forward = params['right_forward_pin']
        self.right_backward = params['right_backward_pin']

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_forward, GPIO.OUT)
        GPIO.setup(self.left_backward, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT)
        GPIO.setup(self.right_backward, GPIO.OUT)

        # PWM (1 kHz)
        self.lf_pwm = GPIO.PWM(self.left_forward, 1000)
        self.lb_pwm = GPIO.PWM(self.left_backward, 1000)
        self.rf_pwm = GPIO.PWM(self.right_forward, 1000)
        self.rb_pwm = GPIO.PWM(self.right_backward, 1000)

        self.lf_pwm.start(0)
        self.lb_pwm.start(0)
        self.rf_pwm.start(0)
        self.rb_pwm.start(0)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive equations
        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        # Convert to RPM
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        # Clamp
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

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
