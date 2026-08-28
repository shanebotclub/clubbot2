#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from gpiozero import Motor as GpioMotor

def scale_duty(val, min_pwm=0.20):
    """
    Scales normalized PID output (-1.0 to 1.0) to jump over motor deadband.
    Preserves the sign for forward/backward driving.
    """
    if abs(val) < 1e-4:
        return 0.0

    sign = 1.0 if val > 0 else -1.0
    magnitude = abs(val)
    duty = min_pwm + magnitude * (1.0 - min_pwm)
    duty = max(min_pwm, min(1.0, duty))
    
    return sign * duty


class PidMotorController(Node):
    def __init__(self):
        super().__init__('pid_motor_controller')

        # Declare Parameters
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

        self.declare_parameter('min_pwm_left', 0.20)
        self.declare_parameter('min_pwm_right', 0.24)

        self.declare_parameter('pid_left_p', 0.25)
        self.declare_parameter('pid_left_i', 0.02)
        self.declare_parameter('pid_left_d', 0.001)

        self.declare_parameter('pid_right_p', 0.35)
        self.declare_parameter('pid_right_i', 0.02)
        self.declare_parameter('pid_right_d', 0.001)

        # Read Parameters
        self.robot_name = self.get_parameter('robot_name').value
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

        self.min_pwm_left = self.get_parameter('min_pwm_left').value
        self.min_pwm_right = self.get_parameter('min_pwm_right').value

        self.kp_left = self.get_parameter('pid_left_p').value
        self.ki_left = self.get_parameter('pid_left_i').value
        self.kd_left = self.get_parameter('pid_left_d').value

        self.kp_right = self.get_parameter('pid_right_p').value
        self.ki_right = self.get_parameter('pid_right_i').value
        self.kd_right = self.get_parameter('pid_right_d').value

        # Hardware Setup using standard gpiozero Motor
        self.left_motor = GpioMotor(forward=self.left_forward, backward=self.left_backward)
        self.right_motor = GpioMotor(forward=self.right_forward, backward=self.right_backward)

        # Max Angular Velocities (rad/s)
        self.max_w_left = (self.left_max_rpm * 2.0 * math.pi) / 60.0
        self.max_w_right = (self.right_max_rpm * 2.0 * math.pi) / 60.0

        # State tracking
        self.left_tick_count = 0
        self.right_tick_count = 0
        self.prev_left_tick_count = 0
        self.prev_right_tick_count = 0

        self.target_w_left = 0.0
        self.target_w_right = 0.0

        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0

        # Subscriptions
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32MultiArray, '/wheel_ticks', self.encoder_callback, 10)

        # Loop timing
        self.loop_period = 0.05
        self.create_timer(self.loop_period, self.control_loop)

        self.get_logger().info(f"PID Controller Ready for '{self.robot_name}'")

    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            self.left_tick_count = msg.data[0]
            self.right_tick_count = msg.data[1]

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        # Kinematics
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        r = self.wheel_diameter / 2.0
        self.target_w_left = v_left / r
        self.target_w_right = v_right / r

    def control_loop(self):
        dt = self.loop_period

        # 1. Measured wheel speeds
        delta_left = self.left_tick_count - self.prev_left_tick_count
        delta_right = self.right_tick_count - self.prev_right_tick_count

        self.prev_left_tick_count = self.left_tick_count
        self.prev_right_tick_count = self.right_tick_count

        measured_w_left = ((delta_left / self.left_ticks_per_rotation) * 2.0 * math.pi) / dt
        measured_w_right = ((delta_right / self.right_ticks_per_rotation) * 2.0 * math.pi) / dt

        # Stop condition
        if abs(self.target_w_left) < 1e-4 and abs(self.target_w_right) < 1e-4:
            self.left_motor.stop()
            self.right_motor.stop()
            self.integral_left = 0.0
            self.integral_right = 0.0
            self.prev_error_left = 0.0
            self.prev_error_right = 0.0
            return

        # 2. PID Calculations
        err_l = self.target_w_left - measured_w_left
        self.integral_left += err_l * dt
        deriv_l = (err_l - self.prev_error_left) / dt
        self.prev_error_left = err_l

        u_l = (self.kp_left * err_l) + (self.ki_left * self.integral_left) + (self.kd_left * deriv_l)

        err_r = self.target_w_right - measured_w_right
        self.integral_right += err_r * dt
        deriv_r = (err_r - self.prev_error_right) / dt
        self.prev_error_right = err_r

        u_r = (self.kp_right * err_r) + (self.ki_right * self.integral_right) + (self.kd_right * deriv_r)

        # 3. Normalize (-1.0 to 1.0)
        norm_l = max(-1.0, min(1.0, u_l / self.max_w_left))
        norm_r = max(-1.0, min(1.0, u_r / self.max_w_right))

        # 4. Deadband compensation and motor values
        self.left_motor.value = scale_duty(norm_l, min_pwm=self.min_pwm_left)
        self.right_motor.value = scale_duty(norm_r, min_pwm=self.min_pwm_right)

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