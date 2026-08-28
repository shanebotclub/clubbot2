#!/usr me/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from gpiozero import PWMOutputDevice, OutputDevice


def scale_duty(val, min_pwm=0.20):
    """
    Applies deadband compensation and scales normalized motor command (-1.0 to 1.0)
    to duty cycle (min_pwm to 1.0). Returns 0.0 when input velocity is zero.
    """
    if abs(val) < 1e-4:
        return 0.0

    magnitude = abs(val)
    duty = min_pwm + magnitude * (1.0 - min_pwm)
    return max(min_pwm, min(1.0, duty))


class Motor:
    """Helper class to control directional pin pairs via GPIO Zero."""
    def __init__(self, forward_pin: int, backward_pin: int):
        self.forward = PWMOutputDevice(forward_pin)
        self.backward = PWMOutputDevice(backward_pin)

    @property
    def value(self):
        # Positive represents forward duty, negative represents backward duty
        return self.forward.value - self.backward.value

    @value.setter
    def value(self, val: float):
        val = max(-1.0, min(1.0, float(val)))
        if val > 0:
            self.forward.value = val
            self.backward.value = 0.0
        elif val < 0:
            self.forward.value = 0.0
            self.backward.value = abs(val)
        else:
            self.forward.value = 0.0
            self.backward.value = 0.0


class PidMotorController(Node):
    def __init__(self):
        super().__init__('pid_motor_controller')

        # ----------------------------------------------------------------------
        # Declare Parameters with Defaults
        # ----------------------------------------------------------------------
        self.declare_parameter('robot_name', 'aver')
        self.declare_parameter('wheel_diameter', 0.08)
        self.declare_parameter('wheel_base', 0.175)

        self.declare_parameter('left_max_rpm', 60)
        self.declare_parameter('right_max_rpm', 60)

        self.declare_parameter('left_ticks_per_rotation', 620)
        self.declare_parameter('right_ticks_per_rotation', 620)

        self.declare_parameter('left_forward_pin', 18)
        self.declare_parameter('left_backward_pin', 23)
        self.declare_parameter('right_forward_pin', 24)
        self.declare_parameter('right_backward_pin', 25)

        self.declare_parameter('linear_scale', 0.3)
        self.declare_parameter('angular_scale', 2.0)

        # Deadband Compensation Parameters
        self.declare_parameter('min_pwm_left', 0.20)
        self.declare_parameter('min_pwm_right', 0.24)

        # PID Parameters
        self.declare_parameter('pid_left_p', 0.25)
        self.declare_parameter('pid_left_i', 0.02)
        self.declare_parameter('pid_left_d', 0.001)

        self.declare_parameter('pid_right_p', 0.35)
        self.declare_parameter('pid_right_i', 0.02)
        self.declare_parameter('pid_right_d', 0.001)

        # ----------------------------------------------------------------------
        # Fetch Parameters
        # ----------------------------------------------------------------------
        self.robot_name = self.get_parameter('robot_name').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value

        self.left_max_rpm = self.get_parameter('left_max_rpm').value
        self.right_max_rpm = self.get_parameter('right_max_rpm').value

        self.left_ticks_per_rotation = self.get_parameter('left_ticks_per_rotation').value
        self.right_ticks_per_rotation = self.get_parameter('right_ticks_per_rotation').value

        left_fwd_pin = self.get_parameter('left_forward_pin').value
        left_bwd_pin = self.get_parameter('left_backward_pin').value
        right_fwd_pin = self.get_parameter('right_forward_pin').value
        right_bwd_pin = self.get_parameter('right_backward_pin').value

        # Fetch Deadband Parameters
        self.min_pwm_left = self.get_parameter('min_pwm_left').value
        self.min_pwm_right = self.get_parameter('min_pwm_right').value

        # Fetch PID Parameters
        self.kp_left = self.get_parameter('pid_left_p').value
        self.ki_left = self.get_parameter('pid_left_i').value
        self.kd_left = self.get_parameter('pid_left_d').value

        self.kp_right = self.get_parameter('pid_right_p').value
        self.ki_right = self.get_parameter('pid_right_i').value
        self.kd_right = self.get_parameter('pid_right_d').value

        # ----------------------------------------------------------------------
        # Hardware & PID Initialization
        # ----------------------------------------------------------------------
        self.left_motor = Motor(left_fwd_pin, left_bwd_pin)
        self.right_motor = Motor(right_fwd_pin, right_bwd_pin)

        # Derived Max Angular Velocities (rad/s)
        self.max_w_left = (self.left_max_rpm * 2.0 * math.pi) / 60.0
        self.max_w_right = (self.right_max_rpm * 2.0 * math.pi) / 60.0

        # Encoder state tracking
        self.left_tick_count = 0
        self.right_tick_count = 0
        self.prev_left_tick_count = 0
        self.prev_right_tick_count = 0

        # Target Wheel Angular Velocities (rad/s)
        self.target_w_left = 0.0
        self.target_w_right = 0.0

        # PID Integral & Error Accumulators
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0

        # ----------------------------------------------------------------------
        # ROS 2 Subscriptions & Timers
        # ----------------------------------------------------------------------
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, 'left_ticks', self.left_ticks_callback, 10)
        self.create_subscription(Int32, 'right_ticks', self.right_ticks_callback, 10)

        # Main Control Loop Frequency (e.g., 20 Hz -> dt = 0.05s)
        self.loop_period = 0.05
        self.timer = self.create_timer(self.loop_period, self.control_loop)

        self.get_logger().info(
            f"PID Motor Controller Node initialized for robot: '{self.robot_name}' "
            f"(Min PWM Left: {self.min_pwm_left}, Min PWM Right: {self.min_pwm_right})"
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics -> Target wheel angular velocities (rad/s)
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        r = self.wheel_diameter / 2.0
        self.target_w_left = v_left / r
        self.target_w_right = v_right / r

    def left_ticks_callback(self, msg: Int32):
        self.left_tick_count = msg.data

    def right_ticks_callback(self, msg: Int32):
        self.right_tick_count = msg.data

    def control_loop(self):
        dt = self.loop_period

        # 1. Calculate measured wheel angular velocity (rad/s)
        delta_left_ticks = self.left_tick_count - self.prev_left_tick_count
        delta_right_ticks = self.right_tick_count - self.prev_right_tick_count

        self.prev_left_tick_count = self.left_tick_count
        self.prev_right_tick_count = self.right_tick_count

        rotations_left = delta_left_ticks / float(self.left_ticks_per_rotation)
        rotations_right = delta_right_ticks / float(self.right_ticks_per_rotation)

        measured_w_left = (rotations_left * 2.0 * math.pi) / dt
        measured_w_right = (rotations_right * 2.0 * math.pi) / dt

        # 2. Compute PID Control Output for Left Motor
        error_left = self.target_w_left - measured_w_left
        self.integral_left += error_left * dt
        derivative_left = (error_left - self.prev_error_left) / dt
        self.prev_error_left = error_left

        u_left = (self.kp_left * error_left) + (self.ki_left * self.integral_left) + (self.kd_left * derivative_left)

        # 3. Compute PID Control Output for Right Motor
        error_right = self.target_w_right - measured_w_right
        self.integral_right += error_right * dt
        derivative_right = (error_right - self.prev_error_right) / dt
        self.prev_error_right = error_right

        u_right = (self.kp_right * error_right) + (self.ki_right * self.integral_right) + (self.kd_right * derivative_right)

        # 4. Normalize PID output relative to max velocity capabilities (-1.0 to 1.0)
        norm_l = u_left / self.max_w_left
        norm_r = u_right / self.max_w_right

        # 5. Apply per-side deadband compensation scaling & set hardware duty cycles
        if abs(self.target_w_left) < 1e-4:
            self.left_motor.value = 0.0
            self.integral_left = 0.0  # Reset integrator when stopped
        else:
            self.left_motor.value = scale_duty(norm_l, min_pwm=self.min_pwm_left)

        if abs(self.target_w_right) < 1e-4:
            self.right_motor.value = 0.0
            self.integral_right = 0.0  # Reset integrator when stopped
        else:
            self.right_motor.value = scale_duty(norm_r, min_pwm=self.min_pwm_right)


def main(args=None):
    rclpy.init(args=args)
    node = PidMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_motor.value = 0.0
        node.right_motor.value = 0.0
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()