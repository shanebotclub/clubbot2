import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # Create publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Create timer (50 Hz)
        self.timer = self.create_timer(0.02, self.publish_imu_data)

        # Init sensor
        self.sensor = mpu6050(0x68)

        self.get_logger().info("MPU6050 IMU node started")

    def publish_imu_data(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        msg = Imu()

        # Timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = accel['x'] * 9.80665
        msg.linear_acceleration.y = accel['y'] * 9.80665
        msg.linear_acceleration.z = accel['z'] * 9.80665

        # Angular velocity (rad/s)
        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        # Orientation unknown → leave as zeros
        # Nav2 is fine with this

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
