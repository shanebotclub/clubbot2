import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from ros2_control_interfaces.msg import JointCommand
from ros2_control_interfaces.msg import JointState

import RPi.GPIO as GPIO
import math

class ClubbotHardware(Node):
    def __init__(self):
        super().__init__("clubbot_hardware")

        # Motor pins (adjust to your robot)
        self.left_forward = 17
        self.left_backward = 27
        self.right_forward = 23
        self.right_backward = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_forward, GPIO.OUT)
        GPIO.setup(self.left_backward, GPIO.OUT)
        GPIO.setup(self.right_forward, GPIO.OUT)
        GPIO.setup(self.right_backward, GPIO.OUT)

        self.lf_pwm = GPIO.PWM(self.left_forward, 1000)
        self.lb_pwm = GPIO.PWM(self.left_backward, 1000)
        self.rf_pwm = GPIO.PWM(self.right_forward, 1000)
        self.rb_pwm = GPIO.PWM(self.right_backward, 1000)

        self.lf_pwm.start(0)
        self.lb_pwm.start(0)
        self.rf_pwm.start(0)
        self.rb_pwm.start(0)

        # Subscribe to wheel velocity commands
        self.cmd_sub = self.create_subscription(
            JointCommand,
            "/diff_drive_controller/joint_commands",
            self.cmd_callback,
            10
        )

        self.get_logger().info("Clubbot hardware interface ready.")

    def cmd_callback(self, msg):
        # msg.velocity = [left_vel, right_vel]
        left_vel = msg.velocity[0]
        right_vel = msg.velocity[1]

        # Convert rad/s → PWM duty cycle
        duty_l = min(abs(left_vel) * 50, 100)
        duty_r = min(abs(right_vel) * 50, 100)

        # Left motor
        if left_vel >= 0:
            self.lf_pwm.ChangeDutyCycle(duty_l)
            self.lb_pwm.ChangeDutyCycle(0)
        else:
            self.lf_pwm.ChangeDutyCycle(0)
            self.lb_pwm.ChangeDutyCycle(duty_l)

        # Right motor
        if right_vel >= 0:
            self.rf_pwm.ChangeDutyCycle(duty_r)
            self.rb_pwm.ChangeDutyCycle(0)
        else:
            self.rf_pwm.ChangeDutyCycle(0)
            self.rb_pwm.ChangeDutyCycle(duty_r)

    def destroy_node(self):
        self.lf_pwm.stop()
        self.lb_pwm.stop()
        self.rf_pwm.stop()
        self.rb_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()
