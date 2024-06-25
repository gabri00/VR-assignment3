#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Bool
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
		self.turn_pub = rospy.Publisher('/turn', Bool, queue_size=1)

		# Vars for obstacle avoidance
		self.sensor_data = None
		self.vel = 5.0


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def get_sensor_data(self, msg):
		points_list = []
		for point in pc2.read_points(msg, skip_nans=True):
			points_list.append([point[0], point[1]])

		# Convert to numpy array
		self.sensor_data = np.array(points_list, dtype=np.float32)

		turn, front_w = self.compute_weights()

		msg = Bool()

		if front_w < 0.1:
			vel_cmd = np.array([self.vel, 0])
			msg.data = False
		else:
			vel_cmd = np.array([0, turn * self.vel])
			msg.data = True
		
		self.airsim.move_vel(vel_cmd)

		self.turn_pub.publish(msg)

	
	def compute_weights(self):
		with np.errstate(divide='ignore', invalid='ignore'):
			weights = 1.0 / np.square(np.linalg.norm(self.sensor_data, axis=1))
			weights[~np.isfinite(weights)] = 0 # Replace infinities and NaNs with 0

		left_weights  = []
		right_weights = []
		front_weights = []

		for i in range(0, self.sensor_data.shape[0]):
			if self.sensor_data[i][1] < -1: 		# y < -1
				left_weights.append(weights[i])
			elif self.sensor_data[i][1] > 1: 		# y > 1
				right_weights.append(weights[i])
			else: 									# -1 <= y <= 1
				front_weights.append(weights[i])
		# self.__logger.loginfo(f"data: {self.sensor_data}")
		# self.__logger.loginfo(f"f: {front_weights}")
		# self.__logger.loginfo(f"l: {left_weights}")
		# self.__logger.loginfo(f"r: {right_weights}")

		turn = -1 if (sum(right_weights) - sum(left_weights)) > 0 else 1
		return turn, sum(front_weights)



def main():
	LocalPlanner('local_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
