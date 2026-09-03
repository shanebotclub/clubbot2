#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from gpiozero import Motor as GpioMotor
from rcl_interfaces.msg import SetParametersResult


def scale_duty(val, min_pwm=0.20):
    """Scales normalized output past static friction deadband while maintaining sign."""
    if abs(val) < 0.01:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    magnitude = abs(val)
    duty = min_pwm + magnitude * (1.0 - min_pwm)
    duty = max(min_pwm, min(1.0, duty))
    return sign * duty


class PidMotorController(Node):
    def __init__(self):
        super().__init__('pid_motor_controller')

        # Declare parameters
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('wheel_diameter', 0.08)
        self.declare_parameter('wheel_base', 0.175)
        self.declare_parameter('left_max_rpm', 107.0)
        self.declare_parameter('right_max_rpm', 107.0)
        self.declare_parameter('left_ticks_per_rotation', 620.0)
        self.declare_parameter('right_ticks_per_rotation', 620.0)
        self.declare_parameter('left_forward_pin', 18)
        self.declare_parameter('left_backward_pin', 23)
        self.declare_parameter('right_forward_pin', 24)
        self.declare_parameter('right_backward_pin', 25)
        self.declare_parameter('linear_scale', 0.4)
        self.declare_parameter('angular_scale', 2.0)
        self.declare_parameter('min_pwm_left', 0.20)
        self.declare_parameter('min_pwm_right', 0.24)
        self.declare_parameter('pid_left_p', 0.25)
        self.declare_parameter('pid_left_i', 0.02)
        self.declare_parameter('pid_left_d', 0.001)
        self.declare_parameter('pid_right_p', 0.35)
        self.declare_parameter('pid_right_i', 0.02)
        self.declare_parameter('pid_right_d', 0.001)
        self.declare_parameter('cmd_timeout', 0.6)  # Timeout in seconds to bridge key-repeat gaps

        # Cache parameters locally once on startup
        self.load_cached_parameters()

        # Dynamic parameter callback setup
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Initialize Motor Hardware
        left_forward = int(self.get_parameter('left_forward_pin').value)
        left_backward = int(self.get_parameter('left_backward_pin').value)
        right_forward = int(self.get_parameter('right_forward_pin').value)
        right_backward = int(self.get_parameter('right_backward_pin').value)

        self.left_motor = GpioMotor(forward=left_forward, backward=left_backward)
        self.right_motor = GpioMotor(forward=right_forward, backward=right_backward)

        # State Variables
        self.left_tick_count = 0
        self.right_tick_count = 0
        self.prev_left_tick_count = 0
        self.prev_right_tick_count = 0

        self.target_rpm_left = 0.0
        self.target_rpm_right = 0.0
        self.actual_rpm_left = 0.0
        self.actual_rpm_right = 0.0

        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0

        # Watchdog timing state
        self.last_cmd_vel_time = time.time()

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32MultiArray, '/wheel_ticks', self.encoder_callback, 10)

        # Control loop at 20 Hz (0.05s)
        self.loop_period = 0.05
        self.last_loop_time = time.time()
        self.create_timer(self.loop_period, self.control_loop)

        self.get_logger().info("PID Motor Controller Initialized with Feed-Forward, Watchdog & Cached Parameters.")

    def load_cached_parameters(self):
        """Reads parameters once into memory to eliminate runtime parameter retrieval overhead."""
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.left_max_rpm = float(self.get_parameter('left_max_rpm').value)
        self.right_max_rpm = float(self.get_parameter('right_max_rpm').value)
        self.ticks_per_rev_l = float(self.get_parameter('left_ticks_per_rotation').value)
        self.ticks_per_rev_r = float(self.get_parameter('right_ticks_per_rotation').value)
        self.min_pwm_l = float(self.get_parameter('min_pwm_left').value)
        self.min_pwm_r = float(self.get_parameter('min_pwm_right').value)
        self.kp_l = float(self.get_parameter('pid_left_p').value)
        self.ki_l = float(self.get_parameter('pid_left_i').value)
        self.kd_l = float(self.get_parameter('pid_left_d').value)
        self.kp_r = float(self.get_parameter('pid_right_p').value)
        self.ki_r = float(self.get_parameter('pid_right_i').value)
        self.kd_r = float(self.get_parameter('pid_right_d').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

    def parameters_callback(self, params):
        """Dynamically updates cached parameter variables when parameters change externally."""
        for param in params:
            if param.name == 'linear_scale':
                self.linear_scale = float(param.value)
            elif param.name == 'angular_scale':
                self.angular_scale = float(param.value)
            elif param.name == 'min_pwm_left':
                self.min_pwm_l = float(param.value)
            elif param.name == 'min_pwm_right':
                self.min_pwm_r = float(param.value)
            elif param.name == 'pid_left_p':
                self.kp_l = float(param.value)
            elif param.name == 'pid_left_i':
                self.ki_l = float(param.value)
            elif param.name == 'pid_left_d':
                self.kd_l = float(param.value)
            elif param.name == 'pid_right_p':
                self.kp_r = float(param.value)
            elif param.name == 'pid_right_i':
                self.ki_r = float(param.value)
            elif param.name == 'pid_right_d':
                self.kd_r = float(param.value)
            elif param.name == 'cmd_timeout':
                self.cmd_timeout = float(param.value)
        return SetParametersResult(successful=True)

    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.left_tick_count = msg.data[0]
            self.right_tick_count = msg.data[1]

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = time.time()

        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        v_l = v - w * (self.wheel_base / 2.0)
        v_r = v + w * (self.wheel_base / 2.0)

        rpm_l = (v_l / (math.pi * self.wheel_diameter)) * 60.0
        rpm_r = (v_r / (math.pi * self.wheel_diameter)) * 60.0

        self.target_rpm_left = max(min(rpm_l, self.left_max_rpm), -self.left_max_rpm)
        self.target_rpm_right = max(min(rpm_r, self.right_max_rpm), -self.right_max_rpm)

    def control_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now

        if dt <= 0.0:
            return

        # Watchdog Timeout Check: Stop motors only if key releases beyond threshold gap
        if (now - self.last_cmd_vel_time) > self.cmd_timeout:
            self.target_rpm_left = 0.0
            self.target_rpm_right = 0.0

        # 1. Calculate Raw RPM
        delta_left = self.left_tick_count - self.prev_left_tick_count
        delta_right = self.right_tick_count - self.prev_right_tick_count
        self.prev_left_tick_count = self.left_tick_count
        self.prev_right_tick_count = self.right_tick_count

        raw_rpm_left = (delta_left / self.ticks_per_rev_l) / dt * 60.0
        raw_rpm_right = (delta_right / self.ticks_per_rev_r) / dt * 60.0

        # 2. Continuous EMA Low-Pass Filter (Alpha = 0.1 for high smoothing)
        alpha = 0.1
        self.actual_rpm_left = (alpha * raw_rpm_left) + ((1.0 - alpha) * self.actual_rpm_left)
        self.actual_rpm_right = (alpha * raw_rpm_right) + ((1.0 - alpha) * self.actual_rpm_right)

        # 3. Clean Stop Logic
        if abs(self.target_rpm_left) < 0.5 and abs(self.target_rpm_right) < 0.5:
            self.left_motor.stop()
            self.right_motor.stop()
            self.integral_left = 0.0
            self.integral_right = 0.0
            self.prev_error_left = 0.0
            self.prev_error_right = 0.0
            return

        # 4. Normalized Errors
        err_l = (self.target_rpm_left - self.actual_rpm_left) / self.left_max_rpm
        err_r = (self.target_rpm_right - self.actual_rpm_right) / self.right_max_rpm

        # 5. Integrals & Derivatives
        self.integral_left = max(min(self.integral_left + err_l * dt, 0.2), -0.2)
        self.integral_right = max(min(self.integral_right + err_r * dt, 0.2), -0.2)

        deriv_l = (err_l - self.prev_error_left) / dt
        deriv_r = (err_r - self.prev_error_right) / dt

        self.prev_error_left = err_l
        self.prev_error_right = err_r

        # 6. Compute Feed-Forward + PID Output
        ff_l = self.target_rpm_left / self.left_max_rpm
        ff_r = self.target_rpm_right / self.right_max_rpm

        u_l = ff_l + (self.kp_l * err_l) + (self.ki_l * self.integral_left) + (self.kd_l * deriv_l)
        u_r = ff_r + (self.kp_r * err_r) + (self.ki_r * self.integral_right) + (self.kd_r * deriv_r)

        norm_l = max(-1.0, min(1.0, u_l))
        norm_r = max(-1.0, min(1.0, u_r))

        # 7. Motor Command Output
        self.left_motor.value = scale_duty(norm_l, min_pwm=self.min_pwm_l)
        self.right_motor.value = scale_duty(norm_r, min_pwm=self.min_pwm_r)

    def destroy_node(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PidMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()