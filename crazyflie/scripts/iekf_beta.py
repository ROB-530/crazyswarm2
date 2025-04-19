#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Quaternion, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration

class IEKF(Node):
    def __init__(self):
        super().__init__("iekf")
        # parameters
        self.declare_parameter('imu_topic',        '/cf_2/imu')
        self.declare_parameter('pose_frame',       'cf_1/iekf_pose')  # TF frame for RF sensor input
        self.declare_parameter('odom_topic',       '/cf_2/odom')
        self.declare_parameter('iekf_output_topic','/cf_2/iekf_pose')

        imu_topic    = self.get_parameter("imu_topic").value
        self.pose_frame = self.get_parameter("pose_frame").value
        odom_topic    = self.get_parameter("odom_topic").value
        output_topic  = self.get_parameter("iekf_output_topic").value

        # subscriptions
        self.imu_sub  = self.create_subscription(Imu, imu_topic, self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        # publishers & broadcaster
        self.pose_pub       = self.create_publisher(Odometry, output_topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF listener
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # timers
        self.ekf_timer       = self.create_timer(0.002, self.timer_callback)
        self.rf_sensor_timer = self.create_timer(0.1,   self.rf_sensor_timer_callback)  # 10 Hz

        # state initialization
        self.R = np.eye(3)
        self.v = np.zeros((3,1))
        self.p = np.zeros((3,1))
        self.X = self.compose_state_matrix(self.R, self.v, self.p)
        self.P = np.eye(9) * 1.0
        self.Q = np.eye(9) * 1.0
        self.N = np.eye(3) * 1.0
        self.g = np.array([[0],[0],[-9.81]])
        self.prev_t = None

        self.get_logger().info("IEKF initialized with RF sensor timer at 10 Hz")

    def imu_callback(self, imu_msg: Imu):
        # propagation step unchanged
        omega = np.array([[imu_msg.angular_velocity.x],
                          [imu_msg.angular_velocity.y],
                          [imu_msg.angular_velocity.z]])
        a = 9.81 * np.array([[imu_msg.linear_acceleration.x],
                              [imu_msg.linear_acceleration.y],
                              [imu_msg.linear_acceleration.z]])
        curr_t = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        if self.prev_t is None:
            self.prev_t = curr_t
            return
        dt = curr_t - self.prev_t
        self.prev_t = curr_t
        if dt <= 0:
            return
        R_delta = self.exp_so3(omega * dt)
        self.R = self.R @ R_delta
        self.v = (self.R @ a + self.g) * dt
        self.p = self.p + self.v * dt + 0.5 * (self.R @ a + self.g) * dt**2
        Adg = self.adjoint_SE3(R_delta)
        self.P = Adg @ self.P @ Adg.T + self.Q

    def rf_sensor_timer_callback(self):
        # 10 Hz RF sensor update: grab latest transform cf_1/iekf_pose → cf_2
        try:
            tf = self.tf_buffer.lookup_transform(
                'cf_2/iekf_pose',                  # target frame
                self.pose_frame,         # source frame
                Time(),                  # zero time → latest available
                timeout=Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF {self.pose_frame}->cf_2 lookup failed: {e}")
            return

        # extract translation & rotation
        trans = tf.transform.translation
        t_arr = np.array([[trans.x], [trans.y], [trans.z]])
        q = tf.transform.rotation
        R_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # measurement in cf_2: origin of pose_frame in cf_2
        y = t_arr

        # IEKF update step
        b_k = self.R.T @ y - self.p
        H = np.zeros((3,9))
        H[:,6:9] = np.eye(3)
        S = H @ self.P @ H.T + self.N
        K = self.P @ H.T @ inv(S)

        xi = K @ b_k
        xi_theta = xi[0:3].reshape(3,1)
        xi_v     = xi[3:6].reshape(3,1)
        xi_p     = xi[6:9].reshape(3,1)

        # apply update
        self.R = self.exp_so3(xi_theta) @ self.R
        self.v = self.v + xi_v
        self.p = self.p + xi_p
        self.X = self.compose_state_matrix(self.R, self.v, self.p)

        # correct covariance
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.N @ K.T

    def timer_callback(self):
        # publish current state to cf_2 frame
        r_mat = R.from_matrix(self.R)
        quat = r_mat.as_quat()
        p = Point(x=float(self.p[0]), y=float(self.p[1]), z=float(self.p[2]))
        q = Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
        # broadcast tf to cf_2
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/map'
        t.child_frame_id  = 'cf_2/iekf_pose'
        t.transform.translation = Vector3(x = float(self.p[0]),y = float(self.p[1]),z = float(self.p[2]))
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        # publish odometry
        twist = Twist()
        twist.linear.x = float(self.v[0]); twist.linear.y = float(self.v[1]); twist.linear.z = float(self.v[2])
        pose = Pose(orientation=q, position=p)
        pos_cov = self.P[6:9,6:9]; rot_cov = self.P[0:3,0:3]
        pose_cov = PoseWithCovariance(pose=pose, covariance=np.block([[pos_cov, np.zeros((3,3))],[np.zeros((3,3)), rot_cov]]).flatten().tolist())
        twist_cov = TwistWithCovariance(twist=twist, covariance=np.block([[self.P[3:6,3:6], np.zeros((3,3))],[np.zeros((3,3)), np.zeros((3,3))]]).flatten().tolist())
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = '/odom'
        odom_msg.child_frame_id = 'cf_2/iekf_pose'
        odom_msg.pose = pose_cov
        odom_msg.twist = twist_cov
        self.pose_pub.publish(odom_msg)
        
    def odom_callback(self, odom_msg: Odometry):
        pass

    def compose_state_matrix(self, R, v, p):
        X = np.eye(5)
        X[0:3,0:3] = R; X[0:3,3:4] = v; X[0:3,4:5] = p
        return X
    def exp_so3(self, phi):
        norm = np.linalg.norm(phi)
        if norm < 1e-6: return np.eye(3) + self.skew(phi)
        axis = phi/norm; K = self.skew(axis)
        return np.eye(3)+np.sin(norm)*K+(1-np.cos(norm))*(K@K)
    def skew(self, v):
        v=v.flatten()
        return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    def adjoint_SE3(self, R):
        adj=np.zeros((9,9));
        adj[0:3,0:3]=R;adj[3:6,3:6]=R;adj[6:9,6:9]=R
        return adj


def main():
    rclpy.init()
    node = IEKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
