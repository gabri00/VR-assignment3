#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

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
		self.obst_threshold = 10.0

		self.sensor_data = None

		# Start control loop every 1 second
		# rospy.Timer(rospy.Duration(1), self.control_loop)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def get_sensor_data(self, msg):
		points_list = []
		for point in pc2.read_points(msg, skip_nans=True):
			points_list.append([point[0], point[1], point[2]])

		# Convert to numpy array
		self.sensor_data = np.array(points_list, dtype=np.float32)

		# Get 3D drone position
		curr_pos = self.airsim.get_drone_position()
		curr_pos.append(0)
		curr_pos = np.array(curr_pos)
		
		distances = np.linalg.norm(self.sensor_data - curr_pos, axis=1)

		# Print distances for debugging
		speed, turn, front_w = self.compute_weights(distances)


		msg = Float64()
		if speed < 1 and front_w > 0:
			msg.data = -1 if turn > 0 else 1
		else:
			msg.data = speed
		self.vel_pub.publish(msg)
	
	
	def compute_weights(self, distances):
		with np.errstate(divide='ignore', invalid='ignore'):
			weights = 1.0 / np.sqrt(distances)
			weights[~np.isfinite(weights)] = 0  # Replace infinities and NaNs with 0

		left_weights  = []
		right_weights = []
		front_weights = []
		for i in range(0,len(self.sensor_data)):
			if self.sensor_data[i][1] < -1:
				left_weights.append(weights[i])
			elif self.sensor_data[i][1] > 1:
				right_weights.append(weights[i])
			else:
				front_weights.append(weights[i])
		# self.__logger.loginfo(f"data: {self.sensor_data}")
		# self.__logger.loginfo(f"f: {front_weights}")
		# self.__logger.loginfo(f"l: {left_weights}")
		# self.__logger.loginfo(f"r: {right_weights}")

		turn = sum(right_weights) - sum(left_weights)
		
		speed = max(0, 3 - abs(turn) + sum(front_weights))
		return speed, turn, sum(front_weights)



def main():
	DroneController('local_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
