#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, NavSatFix
from math import *
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid, Odometry
import pymap3d as pm


class RepublishBag(Node):
	def __init__(self):
		super().__init__('republish_bag')

		self.image_topic = '/map'
		self.resolution = 0.05
		self.rectangle_odom = 5
		
		self.robot_odom_value = Odometry()
		self.odom = Odometry()
		self.gps_data = NavSatFix()
		self.visual_odom = self.create_subscription(Odometry, '/visual_odometry', self.get_odometry, 10)
		self.gps_topic = self.create_subscription(NavSatFix, '/gps', self.get_gps, 10)
		
		self.odom_topic = self.create_publisher(Odometry, '/odometry', 10)
		self.odom_gps_topic = self.create_publisher(Odometry, '/odometry_gps', 10)
		self.matrix_n = np.diag([1, 1, 1])
		self.angle_robot = 0
		self.previous_angle = 0
		self.covariance_value = 0.5
		self.count = 0
		self.previous_time_step = 0
		self.actual_time_step = 0
		self.max_value = 0
		self.count_gps = 0
		self.odom_init_diff_x = 0
		self.odom_init_diff_y = 0
		self.init_diff_x = 0
		self.init_diff_y = 0
		self.init_diff_altitude = 0
		self.odom_gps = Odometry()
		self.gps_pose = Point()

	def get_odometry(self, ros_odom):
		
		# copy the header to have the time information
		self.odom.header = ros_odom.header

		# We use the first value readed by the visual odom to begin the dead-reckoning
		
		if(self.count == 0):

			self.odom.pose.pose.position.x = 0.0
			self.odom.pose.pose.position.y = 0.0
			self.odom.pose.pose.position.x = ros_odom.pose.pose.position.x
			self.odom.pose.pose.position.y = ros_odom.pose.pose.position.y
			# Warning: We fill the field z of the orientation (supposed to be in quaternion) with an euler angle
			(_, _, self.odom.pose.pose.orientation.z) = euler_from_quaternion([ros_odom.pose.pose.orientation.x, 
					ros_odom.pose.pose.orientation.y, ros_odom.pose.pose.orientation.z, ros_odom.pose.pose.orientation.w], 
				 'sxyz')
			self.count = 1
		
		# we populate the covariance vector with 0.5 (default value)
		for i in range(0, len(ros_odom.pose.covariance)):
			self.odom.pose.covariance[i] = 0.5

		for i in range(0, len(ros_odom.pose.covariance)):
			self.odom.twist.covariance[i] = 0.5
		# We change the frame of the odometry (we replace camera by /odom)
		self.odom.header.frame_id = 'odom'
		

		self.angle_robot = - ros_odom.twist.twist.angular.z 

		cos_odom = cos(self.angle_robot)
		sin_odom = sin(self.angle_robot)
		x_odom = ros_odom.twist.twist.linear.x
		y_odom = ros_odom.twist.twist.linear.y

		#Matrix from odom
		matrix_transform = np.array([[cos_odom, -sin_odom, x_odom], 
									[sin_odom,   cos_odom, y_odom], 
									[		0, 		    0, 	    1]])
		
		matrix_np1 = np.dot(self.matrix_n, matrix_transform)
		
		self.odom.pose.pose.position.x = matrix_np1[0, 2]
		self.odom.pose.pose.position.y = matrix_np1[1, 2]
		

		x, y, z, w = quaternion_from_euler(0, 0, acos(matrix_np1[0, 0]))
		self.odom.pose.pose.orientation.x = x
		self.odom.pose.pose.orientation.y = y
		self.odom.pose.pose.orientation.z = z
		self.odom.pose.pose.orientation.w = w
		# Publish on the odom topic when modifications are done
		self.odom_topic.publish(self.odom)
		self.matrix_n = matrix_np1
		

	def get_gps(self, ros_gps):
		
		self.gps_data = ros_gps
		self.gps_data.header = ros_gps.header
		self.gps_data.header.stamp = ros_gps.header.stamp
		latitude = self.gps_data.latitude
		longitude = self.gps_data.longitude
		altitude = self.gps_data.altitude
		if (self.count_gps == 0):
			
			self.init_diff_latitude = latitude
			self.init_diff_longitude = longitude
			self.init_diff_altitude = altitude
			self.count_gps = 1
		# Need if the coordinate are globale
		self.gps_pose.x, self.gps_pose.y, _ = pm.geodetic2enu(latitude, longitude, altitude, self.init_diff_latitude, self.init_diff_longitude, 0)


		#TODO find the issue with the transform of and the broacaster
		#self.odom_gps.header.frame_id = '/gps'
		self.odom_gps.pose.pose.position.x = self.gps_pose.x
		self.odom_gps.pose.pose.position.y = self.gps_pose.y
		for i in range(0, len(ros_gps.position_covariance)):
			self.odom_gps.pose.covariance[i] = self.covariance_value
		self.odom_gps.header.frame_id = 'odom'
		#self.odom_gps.child_frame_id = 'base_link'
		theta = -0.523
		self.odom_gps.pose.pose.position.x = self.gps_pose.x * cos(theta) - self.gps_pose.y * sin(theta)
		self.odom_gps.pose.pose.position.y = self.gps_pose.x * sin(theta) + self.gps_pose.y * cos (theta)

		self.odom_gps_topic.publish(self.odom_gps)

def main(args=None):
    rclpy.init(args=args)

    republish_bag = RepublishBag()
    rclpy.spin(republish_bag)

    # Destroy the node explicitly
    republish_bag.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()