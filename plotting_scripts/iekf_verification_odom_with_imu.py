#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def read_data(bag_path, topics):
    """
    Read Odometry and IMU messages from a ROS 2 bag.
    Returns a dict mapping topic -> dict with keys:
      'times': list of float seconds,
      'positions': list of (x, y, z) or None,
      'speeds': list of float linear speed magnitude (from velocity or accel integration)
    """
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv_opts    = rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                               output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, conv_opts)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    data = {t: {'times': [], 'positions': [], 'speeds': []} for t in topics}
    last_times = {t: None for t in topics}
    last_vel = {t: 0 for t in topics}

    while reader.has_next():
        topic_name, raw_data, _ = reader.read_next()
        if topic_name not in topics:
            continue

        msg_cls = get_message(type_map[topic_name])
        msg = deserialize_message(raw_data, msg_cls)

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        entry = data[topic_name]

        if isinstance(msg, Odometry):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            speed = np.linalg.norm((vx, vy, vz))

            entry['positions'].append((x, y, z))
            entry['speeds'].append(speed)

        elif isinstance(msg, Imu):
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            dt = 0
            if last_times[topic_name] is not None:
                dt = ts - last_times[topic_name]
            last_times[topic_name] = ts

            if dt > 0 and dt < 1.0:
                # crude integration to get velocity
                vx = last_vel[topic_name] + ax * dt
                vy = ay * dt  # or use previous vy
                vz = az * dt
                speed = np.linalg.norm((vx, vy, vz))
                last_vel[topic_name] = vx
            else:
                speed = 0.0

            entry['positions'].append((0, 0, 0))  # placeholder
            entry['speeds'].append(speed)

        entry['times'].append(ts)

    return data

def plot_trajectories(data, topics, output_prefix):
    plt.rcParams.update({'figure.autolayout': True})
    fig1, ax1 = plt.subplots(figsize=(8,8))
    fig2, ax2 = plt.subplots(figsize=(8,4))

    any_plotted = False
    for t in topics:
        times = np.array(data[t]['times'])
        if times.size == 0:
            print(f"[WARN] no messages on {t}, skipping.")
            continue

        pos = np.stack(data[t]['positions'], axis=0)
        spd = np.array(data[t]['speeds'])

        order = np.argsort(times)
        times = times[order]
        pos   = pos[order]
        spd   = spd[order]

        if not np.all(pos == 0):
            ax1.plot(pos[:,0], pos[:,1], label=t)

        ax2.plot(times - times[0], spd, label=t)
        any_plotted = True

    if not any_plotted:
        raise RuntimeError("No data to plot for any topic!")

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('XY Trajectories')
    ax1.axis('equal')
    ax1.legend()
    ax1.grid(True)
    fig1.savefig(f"{output_prefix}_traj.png", dpi=200)

    ax2.set_xlabel('Time [s] since start')
    ax2.set_ylabel('Speed [m/s]')
    ax2.set_title('Linear Speed over Time')
    ax2.legend()
    ax2.grid(True)
    fig2.savefig(f"{output_prefix}_speed.png", dpi=200)

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Compare Odometry and IMU topics in a ROS 2 Humble bag')
    parser.add_argument('--bag',    '-b', required=True,
                        help='Path to the ROS 2 bag directory')
    parser.add_argument('--topics', '-t', nargs=2, required=True,
                        metavar=('TOPIC1','TOPIC2'),
                        help='Odometry and IMU topics to compare')
    parser.add_argument('--output', '-o', default='odom_imu_compare',
                        help='Prefix for output PNG files')
    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        raise FileNotFoundError(f"Bag path not found: {args.bag}")

    rclpy.init()
    data = read_data(args.bag, args.topics)
    rclpy.shutdown()

    plot_trajectories(data, args.topics, args.output)
