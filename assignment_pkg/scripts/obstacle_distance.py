#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int64
from assignment_pkg.msg import DistData
import airsim
import time

from Logger import Logger

class DistanceControl:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()
		
		# Initialize AirSim client
		self.__init_client()
		
		self.dist_pub = rospy.Publisher('distance_data', DistData, queue_size=10)
		
		time.sleep(2)

		rospy.Timer(rospy.Duration(2), self.get_dist_data)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def __init_client(self):
		self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)
		self.client.takeoffAsync().join()


	def get_dist_data(self, event):
		self.dist_front = self.client.getDistanceSensorData('Distance_front').distance
		self.dist_left = self.client.getDistanceSensorData('Distance_left').distance
		self.dist_right = self.client.getDistanceSensorData('Distance_right').distance
		
		# Publish msg
		msg = DistData()
		msg.data_front = self.dist_front
		msg.data_left = self.dist_left
		msg.data_right = self.dist_right
		self.dist_pub.publish(msg)


def main():
	DistanceControl('distance_control_node')
	rospy.spin()


if __name__ == '__main__':
	main()
