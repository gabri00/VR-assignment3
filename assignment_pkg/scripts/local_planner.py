#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

from AirSimWrapper import AirSimWrapper
from Logger import Logger


class DroneController:
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
		self.vel_pub = rospy.Publisher('/vel_cmd', Float64, queue_size=1)

		# Vars for obstacle avoidance
		self.obst_threshold = 15.0

		self.sensor_data = None

		# Start control loop
		self.control_loop()


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def get_sensor_data(self, data):
		self.__logger.loginfo(f"{data.data[0]}")
		self.sensor_data = data.data


	def control_loop(self):
		pass


def main():
	DroneController('local_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
