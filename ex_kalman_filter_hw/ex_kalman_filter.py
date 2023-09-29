#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image, NavSatFix
from math import *

# from tf2_py import tf_transformations
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import pymap3d as pm


class ExKalmanFilter(Node):
    def __init__(self):
        super().__init__('ex_kalman_filter')

        self.visual_odom = Odometry()
        self.visual_odom_topic = self.create_subscription(
            Odometry, '/odometry', self.get_visual_odom_ghisoni, 10)

        self.gps_odom_topic = self.create_subscription(
            Odometry, '/odometry_gps', self.get_gps_ghisoni, 10)

        self.motion_model_topic = self.create_subscription(
            Odometry, '/visual_odometry', self.get_motion_model, 10)

        self.fuse_odom_topic = self.create_publisher(
            Odometry, '/fused_pose_ekf', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.start_fuse_sensor)

        self.fused_odom = Odometry()
        self.fused_odom.header.frame_id = "odom"
        self.matrix_n = np.diag([1, 1, 1])
        self.angle_robot = 0
        self.u_x_odom = 0
        self.u_y_odom = 0
        self.nb_iteration = 0
        self.nb_data_get = 1
        self.cos_odom = 0
        self.sin_odom = 0
        self.odom_gps = Odometry()

        self.At = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])  # noise already made from data

        self.Bt = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

        self.Ht = np.diag([1, 1, 0])

        self.Rt = 1 * np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])

        self.Qt = 600 * np.diag([1, 1, 1])

        self.I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        self.gps_data_received = 0
        self.odom_data_received = 0
        self.motion_data_received = 0

        self.dx_t = 0
        self.dy_t = 0
        self.dtheta_t = 0
        self.mu_tm1 = np.array([1, 1, 1])
        self.variance_tm1 = np.diag([1, 1, 1])

    def get_visual_odom_ghisoni(self, ros_odom):
        # update the odom value
        self.visual_odom = ros_odom
        self.odom_data_received = 1
        self.nb_data_get += 1

    def get_motion_model(self, ros_msg):

        self.angle_robot = - ros_msg.twist.twist.angular.z

        self.cos_odom = cos(self.angle_robot)
        self.sin_odom = sin(self.angle_robot)
        self.u_x_odom = ros_msg.twist.twist.linear.x
        self.u_y_odom = ros_msg.twist.twist.linear.y

        self.motion_data_received = 1

    def get_gps_ghisoni(self, ros_gps):

        self.odom_gps.header = ros_gps.header
        self.odom_gps.header.frame_id = 'odom'

        self.odom_gps.pose.pose.position.x = ros_gps.pose.pose.position.x
        self.odom_gps.pose.pose.position.y = ros_gps.pose.pose.position.y

        self.gps_data_received = 1

    """**************************************************************************************
		This function launches the two phases of the kalman filter if the topics 
		have been published and savec the data for the next iteration.
	**************************************************************************************"""

    def start_fuse_sensor(self):

        # This condition allows to
        if self.gps_data_received != 1 or self.odom_data_received != 1 or self.motion_data_received != 1:
            pass
        else:
            self.motion_data_received = 0
            # set up mu_t
            _, _, yaw = euler_from_quaternion([self.visual_odom.pose.pose.orientation.x, self.visual_odom.pose.pose.orientation.y,
                                              self.visual_odom.pose.pose.orientation.z, self.visual_odom.pose.pose.orientation.w], 'sxyz')

            # Update the motion model
            x_fuse = self.mu_tm1[0]
            y_fuse = self.mu_tm1[1]
            theta_fuse = self.mu_tm1[2]
            cos_fuse = cos(theta_fuse)
            sin_fuse = sin(theta_fuse)
            u_t = np.array([[self.cos_odom,  -self.sin_odom, self.u_x_odom],
                            [self.sin_odom,   self.cos_odom, self.u_y_odom],
                            [0, 		      0, 	    	 1]])

            self.Bt = np.array([[cos_fuse,   -sin_fuse, 		0],
                                [sin_fuse,    cos_fuse, 		0],
                                [0, 	 		 0, 	    1]])

            # Update the z_t matrix (observation model)
            gps_data = np.array(
                [self.odom_gps.pose.pose.position.x, self.odom_gps.pose.pose.position.y, 0])

            z_t = gps_data

            predict_mu_t, predict_variance_t = self.extented_kalman_filter_prediction(
                self.mu_tm1, self.variance_tm1, u_t, z_t)
            mu_t, variance_t = self.extented_kalman_filter_innovation(
                predict_mu_t, predict_variance_t, u_t, z_t)
            print("variance" + str(variance_t))

            self.mu_tm1, self.variance_tm1 = mu_t, variance_t

            # Publish the predicted odom
            self.fused_odom.pose.pose.position.x = mu_t[0]
            self.fused_odom.pose.pose.position.y = mu_t[1]
            qx, qy, qz, qw = quaternion_from_euler(0, 0, mu_t[2])
            self.fused_odom.pose.pose.orientation.x = qx
            self.fused_odom.pose.pose.orientation.y = qy
            self.fused_odom.pose.pose.orientation.z = qz
            self.fused_odom.pose.pose.orientation.w = qw
            self.fuse_odom_topic.publish(self.fused_odom)

            print("nb iteration" + str(self.nb_iteration))

    """**************************************************************************************
		This function launches prediction step. It computes the predicted mu_t and 
		predicted_variance_t with the datas from our motion model,
	**************************************************************************************"""

    def extented_kalman_filter_prediction(self, mu_tm1, variance_tm1, u_t, z_t):

        At = self.At
        cos_odom = self.cos_odom
        sin_odom = self.sin_odom

        Bt = self.Bt
        Ht = self.Ht
        Rt = self.Rt
        Qt = self.Qt
        odom_x = self.odom_gps.pose.pose.position.x
        odom_y = self.odom_gps.pose.pose.position.y
        odom_theta = 0
        # We predict our postion and the variance according to our motion model and our previous state

        mu_tm1_33 = np.array([[0, 0, mu_tm1[0]],
                              [0, 0, mu_tm1[1]],
                              [0, 0, 		0]])

        # Compute the Jacobian Matrix
        Gt = self.get_G_matrix(mu_tm1[2])
        predict_mu_t_33 = np.dot(At, mu_tm1_33) + np.dot(Bt, u_t)
        predict_mu_t = np.array(
            [predict_mu_t_33[0, 2], predict_mu_t_33[1, 2], acos(predict_mu_t_33[0, 0])])
        predict_variance_t = np.dot(np.dot(Gt, variance_tm1), Gt.T) + Rt

        return predict_mu_t, predict_variance_t

    """**************************************************************************************
		This function launches the innovation phase of the Extended Kalman Filter
		In this part, our program will compute the Kalman Gain and apply the correction
		on the mu_t and variance_t

	**************************************************************************************"""

    def extented_kalman_filter_innovation(self, predict_mu_t, predict_variance_t, u_t, z_t):

        At = self.At
        Bt = self.Bt
        Ht = self.Ht
        Rt = self.Rt
        Qt = self.Qt

        # Innovation
        Kt_1st_part = np.dot(predict_variance_t, Ht.T)
        Kt_2nd_part = np.linalg.inv(
            np.dot(np.dot(Ht, predict_variance_t), Ht.T) + Qt)
        Kt = np.dot(Kt_1st_part, Kt_2nd_part)
        print(Kt)

        mu_t = predict_mu_t + np.dot(Kt, (z_t - np.dot(Ht.T, predict_mu_t)))
        variance_t = np.dot(self.I - np.dot(Kt, Ht), predict_variance_t)

        self.nb_iteration += 1
        return mu_t, variance_t

    """**************************************************************************************
		This function allows to comptute the Jacobian matrix G
	**************************************************************************************"""

    def get_G_matrix(self, theta):
        cos_theta = cos(theta)
        sin_theta = sin(theta)
        G = np.array([[cos_theta,  0, -sin_theta],
                      [sin_theta,  0,  cos_theta],
                      [0, 	   0, 	       1]])
        return G


def main(args=None):
    rclpy.init(args=args)

    ex_kalman_filter = ExKalmanFilter()
    rclpy.spin(ex_kalman_filter)

    # Destroy the node explicitly
    ex_kalman_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
