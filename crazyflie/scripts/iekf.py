#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.linalg import expm
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

class IEKF(Node):
    def __init__(self):
        super().__init__("iekf")

        self.declare_parameters(
            "",
            [
                ("imu_topic", "/cf_1/imu"),
                ("pose_topic", "/cf_1/pose"),
                ("odom_topic", "/cf_1/odom"),
                ("iekf_output_topic", "/cf_1/iekf_pose")
            ]
        )

        imu_topic = self.get_parameter("imu_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        output_topic = self.get_parameter("iekf_output_topic").value

        # subscribers
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 1)
        self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        # publishers
        self.pose_pub = self.create_publisher(Odometry, output_topic, 1)

        # timers
        self.ekf_timer = self.create_timer(0.2, self.timer_callback)

        self.R = np.eye(3)                                # Rotation matrix (orientation)
        self.v = np.zeros((3, 1))                         # Velocity
        self.p = np.zeros((3, 1))                         # Position
        self.X = self.compose_state_matrix(self.R, self.v, self.p)  # Combined state matrix

        self.P = np.eye(9) * 0.1                          # Error state covariance (9x9)
        self.Q = np.eye(9) * 0.01                         # Process noise covariance (9x9)
        self.N = np.eye(3) * 0.1                          # Measurement noise covariance (3x3)

        self.g = np.array([[0], [0], [9.81]])

        

        self.prev_t = None
        self.get_logger().info("IEKF initialized")

    def imu_callback(self, imu_msg: Imu):
        """
        Propagation step using IMU measurements (angular velocity and acceleration)
        """
        omega = np.array([[imu_msg.angular_velocity.x], 
                          [imu_msg.angular_velocity.y], 
                          [imu_msg.angular_velocity.z]])
        
        a = -9.81*np.array([[imu_msg.linear_acceleration.x], 
                      [imu_msg.linear_acceleration.y], 
                      [imu_msg.linear_acceleration.z]])
        
        # Get current time
        curr_t = imu_msg.header.stamp.sec + (imu_msg.header.stamp.nanosec * 1e-9)
        
        # Skip first iteration to establish time reference
        if self.prev_t is None:
            self.prev_t = curr_t
            return
            
        dt = curr_t - self.prev_t
        self.prev_t = curr_t
        
        if dt <= 0:
            return
        
        # PROPAGATION STEP (from the formulation in section 7.2.1)
        omega_dt = omega * dt
        R_delta = self.exp_so3(omega_dt)
        R_new = self.R @ R_delta
        
        v_new = self.v + (self.R @ a + self.g) * dt
        
        p_new = self.p + self.v * dt + 0.5 * (self.R @ a + self.g) * dt**2
        
        # Update state
        self.R = R_new
        self.v = v_new
        self.p = p_new
        
        self.X = self.compose_state_matrix(self.R, self.v, self.p)
        
        Adg = self.adjoint_SE3(R_delta)
        
        self.P = Adg @ self.P @ Adg.T + self.Q

    def pose_callback(self, pose_msg: PoseStamped):
        """
        Update step using position measurements
        """
        # Extract position measurement
        y = np.array([[pose_msg.pose.position.x], 
                      [pose_msg.pose.position.y], 
                      [pose_msg.pose.position.z]])
        
        b_k = self.R.T @ (y)- self.p #
        
        H = np.zeros((3, 9))
        H[:, 6:9] = np.eye(3)
        S = H @ self.P @ H.T + self.N
        K = self.P @ H.T @ inv(S)
        xi = K @ b_k
        
        xi_theta = xi[0:3].reshape(3, 1)  # Rotation error
        xi_v = xi[3:6].reshape(3, 1)      # Velocity error
        xi_p = xi[6:9].reshape(3, 1)      # Position error
        

        self.R = self.R @ self.exp_so3(xi_theta)
        
        self.v = self.v + xi_v
        
        self.p = self.p + xi_p
        
        self.X = self.compose_state_matrix(self.R, self.v, self.p)
        
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.N @ K.T

    def timer_callback(self):
        """
        Publish the current state estimate
        """
        # Convert rotation matrix to quaternion
        r = R.from_matrix(self.R)
        quat = r.as_quat()  # (x,y,z,w)
        
        p = Point()
        p.x = float(self.p[0])
        p.y = float(self.p[1])
        p.z = float(self.p[2])
        
        q = Quaternion()
        q.x = float(quat[0])
        q.y = float(quat[1])
        q.z = float(quat[2])
        q.w = float(quat[3])
        
        twist = Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        twist.linear.x = float(self.v[0])
        twist.linear.y = float(self.v[1])
        twist.linear.z = float(self.v[2])
        
        pose = Pose()
        pose.orientation = q
        pose.position = p
        
        pos_cov = self.P[6:9, 6:9]
        rot_cov = self.P[0:3, 0:3]
        
        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = pos_cov
        pose_cov[3:6, 3:6] = rot_cov
        
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = self.P[3:6, 3:6]  # Velocity covariance
        
        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = pose
        pose_with_cov.covariance = pose_cov.flatten().tolist()
        
        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = twist
        twist_with_cov.covariance = twist_cov.flatten().tolist()
        
        odom_output = Odometry()
        odom_output.pose = pose_with_cov
        odom_output.twist = twist_with_cov
        
        odom_output.header.stamp = self.get_clock().now().to_msg()
        odom_output.header.frame_id = "odom"
        odom_output.child_frame_id = "base_link"
        
        self.pose_pub.publish(odom_output)

    def odom_callback(self, odom_msg: Odometry):
        """
        Callback for ground truth odometry (for evaluation)
        """
        # Not used in this implementation
        pass
    
    def compose_state_matrix(self, R, v, p):
        """
        Compose the state matrix X in SE_2(3) from rotation, velocity and position
        X = [R v p; 0 1 0; 0 0 1]
        """
        X = np.eye(5)
        X[0:3, 0:3] = R
        X[0:3, 3:4] = v
        X[0:3, 4:5] = p
        return X
    
    def exp_so3(self, phi):
        """
        Exponential map from so(3) to SO(3)
        Input: 3x1 vector phi (rotation vector)
        Output: 3x3 rotation matrix
        """
        norm_phi = np.linalg.norm(phi)
        
        if norm_phi < 1e-6:
            # Small angle approximation
            return np.eye(3) + self.skew(phi)
        
        axis = phi / norm_phi
        angle = norm_phi
        
        K = self.skew(axis)
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        return R
    
    def skew(self, v):
        """
        Skew-symmetric matrix from 3D vector
        Input: 3x1 vector v
        Output: 3x3 skew-symmetric matrix
        """
        v = v.flatten()
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
    
    def adjoint_SE3(self, R):
        """
        Adjoint map for SE(3)
        Input: 3x3 rotation matrix R
        Output: 9x9 adjoint matrix (for the error-state)
        """
        # For the given IEKF formulation with [R, v, p] state:
        # The adjoint maps the error state ξ = [ξ_θ, ξ_v, ξ_p]
        adj = np.zeros((9, 9))
        
        adj[0:3, 0:3] = R
        
        adj[3:6, 3:6] = R
        
        adj[6:9, 6:9] = R
        
        return adj

def main():
    rclpy.init()
    node = IEKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()