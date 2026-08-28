#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from gpiozero import Motor
import math
import time

def scale_duty(duty_val, min_pwm=0.08):
    """Smoothly scale PWM over min_pwm without step-function kicks."""
    if abs(duty_val) < 0.01:
        return 0.0
    sign = 1 if duty_val > 0 else -1
    # Scaled linearly over deadband threshold
    return sign * (min_pwm + (abs(duty_val) * (1.0 - min_pwm)))

class PID:
    def __init__(self, kp, ki, kd, limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        # Tight anti-windup clamping
        self.integral = max(min(self.integral, 0.2), -0.2)

        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        return max(min(output, self.limit), -self.limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.08)
        self.declare_parameter('wheel_base', 0.175)
        self.declare_parameter('left_max_rpm', 60)   # Matched to 60RPM Motor spec
        self.declare_parameter('right_max_rpm', 60)  # Matched to 60RPM Motor spec
        self.declare_parameter('left_ticks_per_rotation', 620)
        self.declare_parameter('right_ticks_per_rotation', 620)
        self.declare_parameter('left_forward_pin', 18)
        self.declare_parameter('left_backward_pin', 23)
        self.declare_parameter('right_forward_pin', 24)
        self.declare_parameter('right_backward_pin', 25)
        self.declare_parameter('linear_scale', 0.4)
        self.declare_parameter('angular_scale', 1.5)

        self.declare_parameter('pid_left_p', 0.15)
        self.declare_parameter('pid_left_i', 0.02)
        self.declare_parameter('pid_left_d', 0.001)
        self.declare_parameter('pid_right_p', 0.15)
        self.declare_parameter('pid_right_i', 0.02)
        self.declare_parameter('pid_right_d', 0.001)

        # Read Parameters
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_max_rpm = self.get_parameter('left_max_rpm').value
        self.right_max_rpm = self.get_parameter('right_max_rpm').value
        self.left_ticks_per_rotation = self.get_parameter('left_ticks_per_rotation').value
        self.right_ticks_per_rotation = self.get_parameter('right_ticks_per_rotation').value

        self.left_forward = self.get_parameter('left_forward_pin').value
        self.left_backward = self.get_parameter('left_backward_pin').value
        self.right_forward = self.get_parameter('right_forward_pin').value
        self.right_backward = self.get_parameter('right_backward_pin').value

        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        lp = self.get_parameter('pid_left_p').value
        li = self.get_parameter('pid_left_i').value
        ld = self.get_parameter('pid_left_d').value
        rp = self.get_parameter('pid_right_p').value
        ri = self.get_parameter('pid_right_i').value
        rd = self.get_parameter('pid_right_d').value

        # Hardware Setup
        self.left_motor = Motor(forward=self.left_forward, backward=self.left_backward)
        self.right_motor = Motor(forward=self.right_forward, backward=self.right_backward)

        # Trackers
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_loop_time = time.time()
        self.left_rpm_actual = 0.0
        self.right_rpm_actual = 0.0
        self.rpm_l_target = 0.0
        self.rpm_r_target = 0.0

        self.left_pid = PID(lp, li, ld)
        self.right_pid = PID(rp, ri, rd)

        # ROS Setup
        self.encoder_sub = self.create_subscription(Int32MultiArray, '/wheel_ticks', self.encoder_callback, 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 10 Hz Control Loop (100ms interval fits 60RPM encoder sampling better)
        self.control_timer = self.create_timer(0.10, self.control_loop)

    def encoder_callback(self, msg):
        self.last_left_ticks = msg.data[0]
        self.last_right_ticks = msg.data[1]

    def cmd_vel_callback(self, msg):
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        self.rpm_l_target = max(min(rpm_l, self.left_max_rpm), -self.left_max_rpm)
        self.rpm_r_target = max(min(rpm_r, self.right_max_rpm), -self.right_max_rpm)

    def control_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        if dt <= 0:
            return

        # Calculate raw speed
        dl = self.last_left_ticks - getattr(self, 'prev_l_ticks', self.last_left_ticks)
        dr = self.last_right_ticks - getattr(self, 'prev_r_ticks', self.last_right_ticks)
        self.prev_l_ticks = self.last_left_ticks
        self.prev_r_ticks = self.last_right_ticks

        raw_rpm_l = (dl / self.left_ticks_per_rotation) / dt * 60.0
        raw_rpm_r = (dr / self.right_ticks_per_rotation) / dt * 60.0

        # Apply Low-Pass Filter (EMA) to eliminate encoder noise stutter
        alpha = 0.3  # Smoothing factor (0.1 = smooth/laggy, 0.9 = noisy/fast)
        self.left_rpm_actual = (alpha * raw_rpm_l) + ((1.0 - alpha) * self.left_rpm_actual)
        self.right_rpm_actual = (alpha * raw_rpm_r) + ((1.0 - alpha) * self.right_rpm_actual)

        # Stop override
        if abs(self.rpm_l_target) < 0.1 and abs(self.rpm_r_target) < 0.1:
            self.left_motor.stop()
            self.right_motor.stop()
            self.left_pid.reset()
            self.right_pid.reset()
            return

        # Calculate normalized error
        err_l = (self.rpm_l_target - self.left_rpm_actual) / self.left_max_rpm
        err_r = (self.rpm_r_target - self.right_rpm_actual) / self.right_max_rpm

        norm_l = self.left_pid.update(err_l, dt)
        norm_r = self.right_pid.update(err_r, dt)

        self.left_motor.value = scale_duty(norm_l, min_pwm=0.22)
        self.right_motor.value = scale_duty(norm_r, min_pwm=0.22)

    def destroy_node(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()