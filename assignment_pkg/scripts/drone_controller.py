#!/usr/bin/env python

import math
import time

import numpy as np
import rospy
from std_msgs.msg import Float64, Bool

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

		# Vars for altitude handling
		self.curr_limit = 0.0
		self.prev_limit = -1.0
		self.alt_threshold = 5.0

		# Vars for obstacle avoidance
		self.obst_threshold = 10.0
		self.yaw_step = 30
		
		# Vars for handling movements in X-Y plane
		self.start_loc = [80040.0, 135390.0, 0.0]
		self.goal_threshold = 5.0
		self.step_size = 5.0

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
		if self.curr_limit != self.prev_limit:
			self.airsim.move_z(self.curr_limit-self.alt_threshold)
			self.prev_limit = self.curr_limit
			self.__logger.logwarn("Changing altitude...")
			time.sleep(4)


	def control_loop(self):
		curr_pos = self.airsim.get_drone_position()
		goal_pos = np.subtract([90470.0, 39540.0, 10.0], self.start_loc) / 100

		yaw = math.atan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0]) * 180 / math.pi
		self.airsim.set_yaw(yaw)

		# Loop while drone is far from goal
		while np.linalg.norm(np.array(curr_pos) - np.array(goal_pos)) > self.goal_threshold:
			# Read distance sensors data
			dist = self.airsim.get_distance_reading('Distance_front')

			# While drone is near an obstacle, turn
			while dist < self.obst_threshold:
				self.airsim.set_yaw(yaw + self.yaw_step)
				self.__logger.logwarn("Obstacle detected, turning...")
				time.sleep(1)
				dist = self.airsim.get_distance_reading('Distance_front')
			
			self.__logger.loginfo(f"Position: {curr_pos[0]}, {curr_pos[1]}, {curr_pos[2]}")

			# Take a step towards the goal while maintaining the yaw angle
			curr_yaw = self.airsim.get_yaw()
			step_vec = [self.step_size * math.cos(curr_yaw), self.step_size * math.sin(curr_yaw), 0.0]
			new_pos = [curr_pos[i] + step_vec[i] for i in range(3)]
			self.airsim.fly_to(new_pos)
			time.sleep(2)

			# Face the goal again by calculating the new yaw angle
			curr_pos = self.airsim.get_drone_position()
			yaw = math.atan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0]) * 180 / math.pi
			self.airsim.set_yaw(yaw)
			time.sleep(1)
		
		self.__logger.loginfo("Goal reached!!!")


def main():
	DroneController('controller')
	rospy.spin()


if __name__ == '__main__':
	main()
