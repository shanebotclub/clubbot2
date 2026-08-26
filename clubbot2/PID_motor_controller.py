#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from gpiozero import Motor
import math
import time


# ============================
# Helper Functions
# ============================

def scale_duty(duty_val, min_pwm=0.35):
    """
    Maps normalized PID output (-1.0 to 1.0) to jump over 
    the motor's physical deadband threshold.
    """
    if abs(duty_val) < 0.01:
        return 0.0
    sign = 1 if duty_val > 0 else -1
    return sign * (min_pwm + (abs(duty_val) * (1.0 - min_pwm)))


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
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        return max(min(output, self.limit), -self.limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ============================
# Motor Controller Node
# ============================

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # ----------------------------
        # Declare parameters
        # ----------------------------
        self.declare_parameter('wheel_diameter', 0.08)
        self.declare_parameter('wheel_base', 0.175)

        self.declare_parameter('left_max_rpm', 107)
        self.declare_parameter('right_max_rpm', 107)

        self.declare_parameter('left_ticks_per_rotation', 620)
        self.declare_parameter('right_ticks_per_rotation', 620)

        self.declare_parameter('left_forward_pin', 18)
        self.declare_parameter('left_backward_pin', 23)
        self.declare_parameter('right_forward_pin', 24)
        self.declare_parameter('right_backward_pin', 25)

        self.declare_parameter('linear_scale', 0.4)
        self.declare_parameter('angular_scale', 1.5)

        # PID gains
        self.declare_parameter('pid_left_p', 0.5)
        self.declare_parameter('pid_left_i', 0.0)
        self.declare_parameter('pid_left_d', 0.0)

        self.declare_parameter('pid_right_p', 0.5)
        self.declare_parameter('pid_right_i', 0.0)
        self.declare_parameter('pid_right_d', 0.0)

        # ----------------------------
        # Load parameters
        # ----------------------------
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

        # PID gains
        lp = self.get_parameter('pid_left_p').value
        li = self.get_parameter('pid_left_i').value
        ld = self.get_parameter('pid_left_d').value

        rp = self.get_parameter('pid_right_p').value
        ri = self.get_parameter('pid_right_i').value
        rd = self.get_parameter('pid_right_d').value

        self.get_logger().info("Motor controller parameters loaded.")

        # ----------------------------
        # gpiozero Motor setup
        # ----------------------------
        self.left_motor = Motor(forward=self.left_forward, backward=self.left_backward)
        self.right_motor = Motor(forward=self.right_forward, backward=self.right_backward)

        # ----------------------------
        # Encoder & Target State
        # ----------------------------
        self.last_left = 0
        self.last_right = 0
        self.last_time = time.time()
        self.left_rpm_actual = 0.0
        self.right_rpm_actual = 0.0

        self.rpm_l_target = 0.0
        self.rpm_r_target = 0.0

        # ----------------------------
        # PID Controllers
        # ----------------------------
        self.left_pid = PID(lp, li, ld)
        self.right_pid = PID(rp, ri, rd)

        # ----------------------------
        # Subscriptions
        # ----------------------------
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.encoder_callback,
            10
        )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ----------------------------
        # Fixed-frequency Control Loop (20 Hz / 50 ms)
        # ----------------------------
        self.control_timer = self.create_timer(0.05, self.control_loop)

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

            self.left_rpm_actual = (dl / self.left_ticks_per_rotation) / dt * 60.0
            self.right_rpm_actual = (dr / self.right_ticks_per_rotation) / dt * 60.0

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

        # Convert linear velocity -> RPM
        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        # Clamp and update target speeds
        self.rpm_l_target = max(min(rpm_l, self.left_max_rpm), -self.left_max_rpm)
        self.rpm_r_target = max(min(rpm_r, self.right_max_rpm), -self.right_max_rpm)

    # ============================
    # Control Loop (20 Hz)
    # ============================

    def control_loop(self):
        # Override control loop to instantly stop motors if target is near zero
        if abs(self.rpm_l_target) < 0.1 and abs(self.rpm_r_target) < 0.1:
            self.left_motor.stop()
            self.right_motor.stop()
            self.left_pid.reset()
            self.right_pid.reset()
            return

        dt = 0.05  # 50 ms loop period

        # PID calculation
        err_l = self.rpm_l_target - self.left_rpm_actual
        err_r = self.rpm_r_target - self.right_rpm_actual

        duty_l = self.left_pid.update(err_l, dt) / 100.0
        duty_r = self.right_pid.update(err_r, dt) / 100.0

        norm_l = max(min(duty_l, 1.0), -1.0)
        norm_r = max(min(duty_r, 1.0), -1.0)

        self.left_motor.value = scale_duty(norm_l, min_pwm=0.35)
        self.right_motor.value = scale_duty(norm_r, min_pwm=0.35)

    # ============================
    # Cleanup
    # ============================

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