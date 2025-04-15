#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('fake_imu_pub')
        self.publisher_ = self.create_publisher(Imu, 'fake_imu', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Imu()
        # Fill in header information
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Compute a simple time-varying angle (in radians)
        theta = self.i * 0.1  # Increasing angle over time
        half_theta = theta / 2.0

        # Set orientation: rotating about z-axis using a quaternion representation.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(half_theta)
        msg.orientation.w = math.cos(half_theta)

        # Set angular velocity: constant rotation around the z-axis.
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.1  # rad/s

        # Set linear acceleration:
        # For a simple trajectory, let's simulate an oscillation in the x-direction.
        msg.linear_acceleration.x = math.sin(theta) * 1.0  # m/s² oscillatory acceleration
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # m/s² representing gravity

        # Optionally, you can fill in covariance arrays if needed. Here they are left as default (zeroed).

        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing IMU: orientation (z=%.2f, w=%.2f), angular_velocity (z=%.2f), linear_acceleration (x=%.2f)'
            % (msg.orientation.z, msg.orientation.w, msg.angular_velocity.z, msg.linear_acceleration.x)
        )
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
