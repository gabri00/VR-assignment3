#!/usr/bin/env python

import math
import time

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
		self.airsim.takeoff()
		self.__logger.loginfo("Drone ready.")

		# Set weather conditions
		if self.weather:
			self.airsim.load_weather(self.weather, self.weather_value)
			self.__logger.loginfo(f"Set weather to '{self.weather}' with value '{self.weather_value}'")

		# Define subscribers
		rospy.Subscriber('/elevation_limit', Float64, self.update_elevation)
		rospy.Subscriber('/airsim_node/Drone/lidar/Lidar1', PointCloud2, self.get_sensor_data)

		# Vars for altitude handling
		self.curr_limit = 0.0
		self.prev_limit = -1.0
		self.alt_threshold = 5.0

		# Vars for obstacle avoidance
		self.obst_threshold = 15.0
		self.yaw_step = 45
		
		# Vars for handling movements in X-Y plane
		self.start_loc = [81590.0, 129520.0]
		self.goal_threshold = 5.0
		self.step_size = 5.0
		
		self.can_move_xy = False
		self.sensor_data = None

		# Start control loop
		self.control_loop()


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')
		self.weather = rospy.get_param('~weather')
		self.weather_value = rospy.get_param('~weather_value')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def update_elevation(self, data):
		self.curr_limit = -data.data
		self.can_move_xy = False if self.curr_limit != self.prev_limit else True


	def get_sensor_data(self, data):
		self.__logger.loginfo(f"Distance front: {data.front}, right: {data.right}, left: {data.left}")
		self.sensor_data = data


	def compute_vel_cmd(self, start, end, speed):
		dir_vec = np.array(end) - np.array(start)
		dist = np.linalg.norm(dir_vec)

		if dist > 0:
			unit_vec = (dir_vec) / dist
		else:
			unit_vec = np.zeros_like(dir_vec)

		return unit_vec * speed


	def control_loop(self):
		curr_pos = self.airsim.get_drone_position()
		goal_pos = np.subtract([81260.0, 112789.999999], self.start_loc) / 100

		# Loop while drone is far from goal
		while np.linalg.norm(np.array(curr_pos) - np.array(goal_pos)) > self.goal_threshold:
			if self.can_move_xy:
				# Calculate yaw angle to face the goal
				yaw = math.atan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0]) * 180 / math.pi
				self.sensor_data = self.airsim.get_lidar_reading("Lidar1")
				# If drone is near an obstacle, turn
				if self.sensor_data.front < self.obst_threshold:
					turn_sign = 1 if self.sensor_data.right <= self.sensor_data.left else -1
					yaw += turn_sign * self.yaw_step

					self.__logger.logwarn("Obstacle detected, turning...")

				self.airsim.set_yaw(yaw)

				# Take a step towards the goal while maintaining the yaw angle
				# curr_yaw = self.airsim.get_yaw()
				step_vec = [self.step_size * math.cos(math.radians(yaw)), self.step_size * math.sin(math.radians(yaw))]
				new_pos = [curr_pos[i] + step_vec[i] for i in range(2)]

				if np.linalg.norm(np.array(curr_pos) - np.array(new_pos)) < self.sensor_data.front:
					self.airsim.fly_to(new_pos)
				else:
					step_vec = [2 * math.cos(math.radians(yaw)), self.step_size * math.sin(math.radians(yaw))]
					new_pos = [curr_pos[i] + step_vec[i] for i in range(2)]
					self.airsim.fly_to(new_pos)
					

				# Face the goal again by calculating the new yaw angle
				curr_pos = self.airsim.get_drone_position()
			else:
				self.airsim.move_z(self.curr_limit-self.alt_threshold)
				self.prev_limit = self.curr_limit
		
		self.__logger.loginfo("Goal reached!!!")
		self.airsim.land()


def main():
	DroneController('controller')
	rospy.spin()


if __name__ == '__main__':
	main()
