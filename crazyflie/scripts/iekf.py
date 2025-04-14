#!/usr/bin/env python3
import pathlib

import rclpy
<<<<<<< Updated upstream
from rclpy import Parameter
=======
import numpy as np
from scipy.linalg import expm
>>>>>>> Stashed changes
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class IEKF(Node):
    def __init__(self):
        super().__init__("iekf")

        self.declare_parameters(
            "",
            [
                ("imu_topic", Parameter.Type.STRING),
                ("pose_topic", Parameter.Type.STRING),
                ("odom_topic", Parameter.Type.STRING),
                ("iekf_output_topic", Parameter.Type.STRING)
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


        # parameters and system
        self.Phi = np.eye(3)               # state transtion matrix for orientation
        self.Q = 1e-4*np.eye(3)            # gyroscope noise covariance
        self.N = 1e-4*np.eye(3)            # accelerometer noise covariance
        self.f = orientation_process_model              # process model for orientation
        self.H = orientation_measurement_Jacobain      # measurement Jacobain

        self.X = np.array([0,0,0])         # state vector for position
        self.cov_pos = 0.1 * np.eye(3)     # state covariance for position
        self.R = np.eye(3)                 # state vector for orientation
        self.cov_orient = 0.1 * np.eye(3)  # state covariance for orientation
        self.vel = np.array([0,0,0])       # velocity tracking

        self.prev_t = 0.0
        self.g = np.zeros((3,1))
        self.g[0,] = 9.81


        #TODO: initialize initial values... also does it make sense to store these separately?
        ##imu data from callback
        # self.imu_omega = None #ang vel
        # self.imu_accel = None #lin accel possibly incorporate for correction?
        # self.imu_orientation = None #quat,

        ##gps data from callback
        # self.gps_pos = None
        # self.gps_orientation = None #quat, but gps doesn't actually provide this
    def imu_callback(self, imu_msg: Imu): #AKA prediction
        # TODO: integrate into IEKF
        # TODO: Couldn't figure out how to get this to publish from gazebo
        # You can see the values if you run the sim and "gz topic --echo --topic /cf_0/imu"
        omega = imu_msg.angular_velocity #gyro
        accel = imu_msg.linear_acceleration #check if (3,1)
        orientation = imu_msg.orientation
        curr_t = imu_msg.header.stamp.sec
        dt = curr_t - self.prev_t
        self.prev_time = curr_t


        phi = omega*dt


        #predict position
        norm = np.linalg.norm()
        phi_hat = wedge(phi)
        # gamma0 = np.eye(3) + np.sin(norm(phi))/norm(phi) *  phi_hat+ (1-np.cos(norm(phi)))/norm(phi)**2 * (phi_hat@phi_hat)
        gamma1 = np.eye(3) + (1-np.cos(norm(phi)))/norm(phi)**2 * phi_hat + ((norm(phi) - np.sin(norm(phi)))/norm(phi)**3) * (phi_hat@phi_hat)
        gamma2 = .5*np.eye(3) + ((norm(phi) - np.sin(norm(phi)))/norm(phi)**3) * phi_hat + ((norm(phi)**2 + 2*np.cos(norm(phi)) -2 ) / (2*norm(phi)**4)) * (phi_hat@phi_hat)

        
        self.X = self.X @ self.vel*dt + self.R@gamma2@accel*dt**2 + .5*self.g*dt**2
        self.vel = self.vel + self.R@ gamma1 @ accel *dt +  self.g*dt
        #predict orientation state
        self.R = self.R @ expm(wedge(phi)) #need to correct omega_g?

        #predict orientation cov

      
        



        return

    def pose_callback(self, pose_msg: PoseStamped): #AKA Correction
        # TODO: integrate into IEKF (treat this as GPS)
        # TODO: this should eventually be slowed down to publish more infrequently
        gps_pos = pose_msg.pose.position

        correction(self)
        return




    def odom_callback(self, odom_msg: Odometry):
        # This is our "ground truth odometry"
        # Use it to compare against our IEKF output
        # TODO: nothing for now
        return

## iekf computation ##
def prediction(self, omega, accel, dt):
    """
    @param omega: gyroscope reading
    @param accel: accelerometer reading
    @param dt:    time step
    """
    #not really doing kalman filtering by directly using orientation...
    self.R = self.f(self.R,omega,dt)
    self.X = 
    self.P = (self.Phi @ self.P @ self.Phi.T + adjoint(self.R)@self.Q@adjoint(self.R).T)
    return

def correction(self, pos, orient):
    """
    @param pos: GPS position
    @param orient: GPS orientation
    """
    H = self.H(g) #meas jacobian
    N = self.R @ self.N @ self.R.T
    S = H @ self.P @ H.T + N
    L = self.P @ H.T @np.linalg.inv(S) # this is the new kalman gain

    # Update state
    # wedge(XY -g) maps from gaussian noise vector to lie algebra. Then expm maps lie algebra to lie group
    self.X =  expm(wedge(L@(self.X@Y.reshape((3,1)) - g.reshape((3,1))))) @ self.X

    # Update Covariance
    self.P = (np.eye(3) - L@H) @ self.P @ (np.eye(3) - L@H).T + L@N@L.T

    return
def imu_motion_model(R, omega, dt):
    """
    @param  R:      State variable (3, 3)
    @param  omega:  gyroscope reading (3,)
    @param  dt:     time step
    @return R_pred: predicted state variable (3, 3)
    """
    R_pred = np.zeros((3,3)) # placeholder
    R_pred = R @ expm(wedge(omega* dt))  #lie algebra map

    return R_pred
def imu_measurement_Jacobain(g):
    """
    @param  g: gravity (3,)
    @return H: measurement Jacobain (3, 3)
    """
    H = np.zeros((3,3)) # placeholder
    # H = np.array([[0,-g[2],g[1]],[g[2],0,-g[0]],[-g[1],g[0],0]])
    H = wedge(g)
    return H
def gps_motion_model(R, omega, dt):
    """
    @param  R:      State variable (3, 3)
    @param  omega:  gyroscope reading (3,)
    @param  dt:     time step
    @return R_pred: predicted state variable (3, 3)
    """
    R_pred = np.zeros((3,3)) # placeholder
    R_pred = R @ expm(wedge(omega* dt))  #lie algebra map

    return R_pred
def gps_measurement_Jacobain(g):
    """
    @param  g: gravity (3,)
    @return H: measurement Jacobain (3, 3)
    """
    H = np.zeros((3,3)) # placeholder
    # H = np.array([[0,-g[2],g[1]],[g[2],0,-g[0]],[-g[1],g[0],0]])
    H = wedge(g)
    return H
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
def adjoint(R):
    """
    Adjoint of SO3 Adjoint (R) = R
    """
    return R

## iekf computation ##
def main():
    rclpy.init()
    node = IEKF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()