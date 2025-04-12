#!/usr/bin/env python3
import pathlib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class IEKF(Node):
    def __init__(self):
        super().__init__("iekf")

        imu_topic = self.get_parameter("imu_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        output_topic = self.get_parameter("iekf_output_topic").value

        # subscribers
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 1)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 1)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 1)

        # publishers
        self.pose_pub = self.create_publisher(Odometry, output_topic, 1)

    def imu_callback(self, imu_msg: Imu):
        # TODO: integrate into IEKF
        # TODO: Couldn't figure out how to get this to publish from gazebo
        # You can see the values if you run the sim and "gz topic --echo --topic /cf_0/imu"
        pass

    def pose_callback(self, pose_msg: PoseStamped):
        # TODO: integrate into IEKF (treat this as GPS)
        # TODO: this should eventually be slowed down to publish more infrequently
        pass

    def odom_topic(self, odom_msg: Odometry):
        # This is our "ground truth odometry"
        # Use it to compare against our IEKF output
        # TODO: nothing for now
        pass


def main():
    rclpy.init()
    node = IEKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()