#!/usr/bin/env python

import numpy as np
import rospy
from assignment_pkg.msg import VelCmd
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from AirSimWrapper import AirSimWrapper
from Logger import Logger


class LocalPlanner:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()
		
		# Initialize AirSim wrapper
		self.airsim = AirSimWrapper(self.host, self.port)

		# Define subscribers
		rospy.Subscriber('/airsim_node/Drone/lidar/Lidar1', PointCloud2, self.get_sensor_data)

		# Define publishers
		self.vel_cmd_pub = rospy.Publisher('/vel_cmd', VelCmd, queue_size=1)

		# Vars for obstacle avoidance
		self.sensor_data = None
		self.vel = 5.0
		self.obst_th = 5.0


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def get_sensor_data(self, msg):
		# Extract lidar points
		points_list = []
		for point in pc2.read_points(msg, skip_nans=True):
			points_list.append([point[0], point[1]])

		# Convert points to numpy array
		self.sensor_data = np.array(points_list, dtype=np.float32)
		self.__logger.loginfo(f"data: {self.sensor_data}")

		# Compute distances from drone
		distances = np.linalg.norm(self.sensor_data, axis=1)

		# Separate distances in L, R, F sides
		l_dist = np.array([])
		r_dist = np.array([])
		f_dist = np.array([])

		for i in range(0, self.sensor_data.shape[0]):
			if self.sensor_data[i][1] < -1: 		# y < -1
				l_dist = np.append(l_dist, distances[i])
			elif self.sensor_data[i][1] > 1: 		# y > 1
				r_dist = np.append(r_dist, distances[i])
			else: 									# -1 <= y <= 1
				f_dist = np.append(f_dist, distances[i])

		# Select only distances too close
		f_dist = f_dist[f_dist < self.obst_th]

		self.__logger.loginfo(f"FRONT: {f_dist}")
		self.__logger.loginfo(f"LEFT: {l_dist}")
		self.__logger.loginfo(f"RIGHT: {r_dist}")

		turn_sgn = 1 if sum(r_dist) > sum(l_dist) else -1

		#self.__logger.loginfo(f"SIGN: {turn_sgn}")

		vel_msg = VelCmd()

		if f_dist.size:
			self.__logger.loginfo(f"OBSTACLE DETECTED")
			vel_msg.vx = 0
			vel_msg.vy = turn_sgn * self.vel
		else:
			vel_msg.vx = self.vel
			vel_msg.vy = 0

		self.vel_cmd_pub.publish(vel_msg)


def main():
	LocalPlanner('local_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
