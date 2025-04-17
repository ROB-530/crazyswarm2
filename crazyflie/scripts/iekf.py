#!/usr/bin/env python3
import pathlib

import rclpy
from rclpy import Parameter
import numpy as np
from scipy.linalg import expm,block_diag #block_diag not accessed
from scipy.spatial.transform import Rotation as R
from numpy.linalg import norm
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Point,Quaternion,Twist,Pose,PoseWithCovariance,TwistWithCovariance
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

class IEKF(Node):
    def __init__(self):
        super().__init__("iekf")

        self.declare_parameters(
            "",
            [
                # ("imu_topic", "/fake_imu"),
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
        # self.pose_sub = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        # publishers
        self.pose_pub = self.create_publisher(Odometry, output_topic, 1)

        #timers
        self.ekf_timer = self.create_timer(0.2, self.timer_callback)

        # parameters and system
        #TODO: initial state needs to be set correctly
        self.Pos = np.zeros((3,1),dtype=np.float64)                                               # state position
        self.R = np.eye(3,dtype=np.float64)                                                       # state orientation
        self.vel = np.zeros((3,1),dtype=np.float64)                                               # velocity
        self.X = np.eye(5,dtype=np.float64)                                                   # combined state vector

        #TODO:block diag of cov_omega,cov_accel,cov_position and these should be fixed values
        self.cov_Pos = 1 * np.eye(3,dtype=np.float64)                                            # covariance for position
        self.cov_accel = 1 * np.eye(3,dtype=np.float64)                                         # covariance for accel
        self.cov_omega = 1 * np.eye(3,dtype=np.float64)                                         # covariance for ang vel

        self.cov_omega_noise= np.random.normal(loc=0,scale=1,size=(3,3))
        self.cov_accel_noise = np.random.normal(loc=0,scale=1,size=(3,3))
        self.cov_Pos_noise = np.random.normal(loc=0,scale=1,size=(3,3))
        self.P = block_diag(self.cov_omega, self.cov_accel,self.cov_Pos)         #combined cov
        self.Q = block_diag(self.cov_omega_noise, self.cov_accel_noise,self.cov_Pos_noise)                      #process noise covariance
        self.N = np.eye(3,dtype=np.float64)                                                       #GPS noise covariance

        self.prev_t = 0.0
        self.g = np.zeros((3,1),dtype=np.float64)           #gravity vector (should be in body frame)
        self.g[2] = -9.81       
        self.init_time_bool = False

    def imu_callback(self, imu_msg: Imu): #prediction
        # You can see the values if you run the sim and "gz topic --echo --topic /cf_0/imu"
        omega = np.array([imu_msg.angular_velocity.x,
                      imu_msg.angular_velocity.y,
                      imu_msg.angular_velocity.z]).reshape((3, 1))
        accel = np.array([imu_msg.linear_acceleration.x,
                      imu_msg.linear_acceleration.y,
                      imu_msg.linear_acceleration.z]).reshape((3, 1))
        
        #skip first iteration to get prev_time
        curr_t = imu_msg.header.stamp.sec + (imu_msg.header.stamp.nanosec * 1e-9)
        if self.init_time_bool == False:
            self.prev_t = curr_t
            self.init_time_bool = True
            return
        dt = curr_t - self.prev_t
        self.prev_time = curr_t

        phi = omega * dt
        norm_phi = norm(phi)
        phi_wedge = wedge(phi)

        if norm_phi < 1e-5:
            gamma0 = np.eye(3) + phi_wedge
            gamma1 = np.eye(3) + 0.5 * phi_wedge
            gamma2 = (1/6) * np.eye(3) + (1/24) * phi_wedge
        else:
            gamma0 = np.eye(3) + (np.sin(norm_phi) / norm_phi) * phi_wedge + \
                    ((1 - np.cos(norm_phi)) / norm_phi**2) * (phi_wedge @ phi_wedge)

            gamma1 = np.eye(3) + ((1 - np.cos(norm_phi)) / norm_phi**2) * phi_wedge + \
                    ((norm_phi - np.sin(norm_phi)) / norm_phi**3) * (phi_wedge @ phi_wedge)

            gamma2 = 0.5 * np.eye(3) + ((norm_phi - np.sin(norm_phi)) / norm_phi**3) * phi_wedge + \
                    ((norm_phi**2 + 2 * np.cos(norm_phi) - 2) / (2 * norm_phi**4)) * (phi_wedge @ phi_wedge)

        #predict position
        self.Pos = self.Pos + self.vel*dt + self.R@gamma2@accel*dt**2 + .5*self.g*dt**2
        self.vel = self.vel + self.R@ gamma1 @ accel *dt +  self.g*dt

        #predict orientation state
        self.R = self.R @ expm(wedge(phi))

        #predict cov
        stm11 = gamma0.T
        stm21 = -stm11@wedge(gamma1@accel)*dt
        stm31 = -stm11@wedge(gamma2@accel)*dt**2
        # stm22 = stm11
        stm32 = stm11*dt
        # stm33 = stm11
        stm = np.block([[stm11,np.zeros((3,3)),np.zeros((3,3))],[stm21,stm11,np.zeros((3,3))],[stm31,stm32,stm11]])
        self.P = stm@self.P@stm.T + self.Q #adj
        return 

    def pose_callback(self, pose_msg: PoseStamped): #correction
        # TODO: this should eventually be slowed down to publish more infrequently

        #position measurement from GPS
        yk = np.array([[pose_msg.pose.position.x], [pose_msg.pose.position.y], [pose_msg.pose.position.z],[0.0],[1.0]]) #(5,1)
        H = np.block([np.zeros((3, 3)), np.zeros((3, 3)), np.eye(3)]) #(3,9) Measurement Jacobian
        S = H @ self.P @ H.T + self.N #(3,3)
        L = self.P @ H.T @ np.linalg.inv(S) #(9,3)
        b = np.zeros((5,1))
        b[-1] = 1
        vk = np.linalg.inv(self.X)@yk - b
        self.X = expm(wedge_error(L@vk[:3])) @ self.X

        #correct individual vars
        self.R = self.X[0:3, 0:3]
        self.vel = self.X[0:3, 3].reshape((3,1))
        self.Pos = self.X[0:3, 4].reshape((3,1))
        return
    
    def timer_callback(self):

        self.get_logger().info("me try publish")

        r = R.from_matrix(self.R)
        quat = r.as_quat() #should be scalar last (x,y,z,w)
        p = Point()
        p.x = float(self.Pos[0])
        p.y = float(self.Pos[1])
        p.z = float(self.Pos[2])
        q = Quaternion()
        q.x = float(quat[0])
        q.y = float(quat[1])
        q.z = float(quat[2])
        q.w = float(quat[3])
        cov = np.zeros((6,6)).flatten()

        twist = Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        twist.linear.x = float(self.vel[0])
        twist.linear.y = float(self.vel[1])
        twist.linear.z = float(self.vel[2])

        pose = Pose()
        pose.orientation = q
        pose.position = p

        pose_with_cov = PoseWithCovariance()
        pose_with_cov.pose = pose
        pose_with_cov.covariance = cov #set to zero for now

        twist_with_cov = TwistWithCovariance()
        twist_with_cov.twist = twist
        twist_with_cov.covariance = cov #set to zero for now

        odom_output = Odometry()
        odom_output.pose = pose_with_cov
        odom_output.twist = twist_with_cov

        #TODO: fill header
        odom_output.header.stamp = self.get_clock().now().to_msg()
        odom_output.header.frame_id = "odom"

        self.pose_pub.publish(odom_output)
        self.get_logger().info("me publish")


    def odom_callback(self, odom_msg: Odometry):
        # This is our "ground truth odometry"
        # Use it to compare against our IEKF output
        # TODO: nothing for now

        return

def wedge(phi):
    """
    R^3 vector to so(3) matrix
    @param  phi: R^3
    @return Phi: so(3) matrix
    """
    phi = phi.squeeze()
    Phi = np.array([[0, -phi[2], phi[1]],
                    [phi[2], 0, -phi[0]],
                    [-phi[1], phi[0], 0]])
    return Phi

def wedge_se2_3(R, v, p):
    """
    R matrix, v, p to se2(3) matrix
    @param  R: Rotation matrix
    @param  v: velocity vector(3,1)
    @param  p: position vector(3,1)
    @return X: se2(3) matrix
    """
    X = block_diag(R,np.ones(1),np.ones(1)) #(5,5)
    X[0:3,3] = v.squeeze()
    X[0:3,4] = p.squeeze()
    return X
def wedge_error(v):
    rot_err = v[0:3].reshape((3,1))       # rotation error
    vel_err = v[3:6].reshape((3,1))       # velocity error
    pos_err = v[6:9].reshape((3,1))       # position error

    rot_err = wedge(rot_err)

    return wedge_se2_3(rot_err,vel_err,pos_err)

def adjoint(R):
    """
    Adjoint of SO3 Adjoint (R) = R
    """
    return R

def main():
    rclpy.init()
    node = IEKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()