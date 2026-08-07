#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import math
import time


# ============================
# PID Controller
# ============================

class PID:
    def __init__(self, kp, ki, kd, limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        return max(min(output, self.limit), -self.limit)


# ============================
# Motor Controller Node
# ============================

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters
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

        # Load parameters
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

        self.get_logger().info("Motor controller parameters loaded.")

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_forward, GPIO.OUT)
        GPIO.setup(self.left_backward, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT)
        GPIO.setup(self.right_backward, GPIO.OUT)

        # PWM setup
        self.lf_pwm = GPIO.PWM(self.left_forward, 1000)
        self.lb_pwm = GPIO.PWM(self.left_backward, 1000)
        self.rf_pwm = GPIO.PWM(self.right_forward, 1000)
        self.rb_pwm = GPIO.PWM(self.right_backward, 1000)

        self.lf_pwm.start(0)
        self.lb_pwm.start(0)
        self.rf_pwm.start(0)
        self.rb_pwm.start(0)

        # Encoder state
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()
        self.left_rpm_actual = 0.0
        self.right_rpm_actual = 0.0

        # PID controllers
        self.left_pid = PID(1.2, 0.3, 0.05)
        self.right_pid = PID(1.2, 0.3, 0.05)

        # Subscribe to encoder ticks
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.encoder_callback,
            10
        )

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("PID motor controller ready.")

    # ============================
    # Encoder callback
    # ============================

    def encoder_callback(self, msg):
        left = msg.data[0]
        right = msg.data[1]

        now = time.time()
        dt = now - self.last_time

        if dt > 0:
            dl = left - self.last_left
            dr = right - self.last_right

            TICKS_PER_REV = 1024  # adjust if needed

            self.left_rpm_actual = (dl / TICKS_PER_REV) / dt * 60.0
            self.right_rpm_actual = (dr / TICKS_PER_REV) / dt * 60.0

        self.last_left = left
        self.last_right = right
        self.last_time = now

    # ============================
    # cmd_vel callback
    # ============================

    def cmd_vel_callback(self, msg):
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        # Differential drive kinematics
        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        # Convert linear velocity → RPM
        rpm_l_target = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r_target = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        # Clamp
        rpm_l_target = max(min(rpm_l_target, self.left_max_rpm), -self.left_max_rpm)
        rpm_r_target = max(min(rpm_r_target, self.right_max_rpm), -self.right_max_rpm)

        # PID error
        err_l = rpm_l_target - self.left_rpm_actual
        err_r = rpm_r_target - self.right_rpm_actual

        dt = 0.05  # 50 ms loop

        # PID output → duty cycle
        duty_l = self.left_pid.update(err_l, dt)
        duty_r = self.right_pid.update(err_r, dt)

        # Drive left motor
        if duty_l >= 0:
            self.lf_pwm.ChangeDutyCycle(duty_l)
            self.lb_pwm.ChangeDutyCycle(0)
        else:
            self.lf_pwm.ChangeDutyCycle(0)
            self.lb_pwm.ChangeDutyCycle(-duty_l)

        # Drive right motor
        if duty_r >= 0:
            self.rf_pwm.ChangeDutyCycle(duty_r)
            self.rb_pwm.ChangeDutyCycle(0)
        else:
            self.rf_pwm.ChangeDutyCycle(0)
            self.rb_pwm.ChangeDutyCycle(-duty_r)

    # ============================
    # Cleanup
    # ============================

    def destroy_node(self):
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
