#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from assignment_pkg.msg import DistData

from AirSimWrapper import AirSimWrapper
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
		
		# Initialize AirSim wrapper
		self.airsim = AirSimWrapper(self.host, self.port)
		
		self.dist_pub = rospy.Publisher('/sensor_data', DistData, queue_size=1)

		rospy.Timer(rospy.Duration(1), self.set_sensor_data)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def set_dist_data(self, event):		
		# Publish msg
		msg = DistData()
		msg.front = self.airsim.get_distance_reading('Distance_front')
		msg.left = self.airsim.get_distance_reading('Distance_left')
		msg.right = self.airsim.get_distance_reading('Distance_right')
		self.dist_pub.publish(msg)


def main():
	DistanceControl('sensor_data_node')
	rospy.spin()


if __name__ == '__main__':
	main()
