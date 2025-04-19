#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from nav_msgs.msg import Odometry


def read_odometry(bag_path, topics):
    """
    Read Odometry messages from a ROS 2 Humble bag.

    Returns a dict mapping topic -> dict with keys:
      'times': list of float seconds,
      'positions': list of (x, y, z),
      'speeds':    list of float linear speed magnitude.
    """
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    conv_opts = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, conv_opts)

    # map topic name -> msg type
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    data = {t: {'times': [], 'positions': [], 'speeds': []} for t in topics}

    while reader.has_next():
        topic_name, raw_data, _ = reader.read_next()
        if topic_name not in topics:
            continue

        msg_cls = get_message(type_map[topic_name])
        odom = deserialize_message(raw_data, msg_cls)

        ts = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z

        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z
        # use vertical speed; switch to magnitude if desired
        speed = np.linalg.norm([vx, vy, vz])

        entry = data[topic_name]
        entry['times'].append(ts)
        entry['positions'].append((x, y, z))
        entry['speeds'].append(speed)

    return data


def plot_trajectories(data, topics, output_prefix):
    plt.rcParams.update({'figure.autolayout': True})

    def chunks(lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    for i, topic_group in enumerate(chunks(topics, 2), start=1):
        group_label = f"Group {i}"
        group_output = f"{output_prefix}_group{i}"

        fig1 = plt.figure(figsize=(8, 8))
        ax1 = fig1.add_subplot(111, projection='3d')
        fig2, ax2 = plt.subplots(figsize=(8, 4))

        any_plotted = False
        for t in topic_group:
            times = np.array(data[t]['times'])
            if times.size == 0:
                print(f"[WARN] no messages on {t}, skipping.")
                continue

            pos = np.stack(data[t]['positions'], axis=0)
            spd = np.array(data[t]['speeds'])

            order = np.argsort(times)
            times = times[order]
            pos = pos[order]
            spd = spd[order]

            ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], label=t)
            ax2.plot(times - times[0], spd, label=t)
            any_plotted = True

        if not any_plotted:
            print(f"[WARN] No data to plot for {group_label}.")
            continue

        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_title(f'{group_label} - 3D Trajectories')
        ax1.legend()
        ax1.grid(True)
        fig1.savefig(f"{group_output}_traj3d.png", dpi=200)

        ax2.set_xlabel('Time [s] since start')
        ax2.set_ylabel('Speed [m/s]')
        ax2.set_title(f'{group_label} - Linear Speed over Time')
        ax2.legend()
        ax2.grid(True)
        fig2.savefig(f"{group_output}_speed.png", dpi=200)

    plt.show()




if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Compare two nav_msgs/Odometry topics in a ROS 2 Humble bag')
    parser.add_argument('--bag', '-b', required=True,
                        help='Path to the ROS 2 bag directory')
    parser.add_argument('--topics', '-t', nargs='+', required=True,
                        metavar='TOPIC',
                        help='Odometry topics to compare (grouped in pairs)')
    parser.add_argument('--output', '-o', default='odometry_compare',
                        help='Prefix for output PNG files')
    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        raise FileNotFoundError(f"Bag path not found: {args.bag}")

    rclpy.init()
    data = read_odometry(args.bag, args.topics)
    rclpy.shutdown()

    plot_trajectories(data, args.topics, args.output)
