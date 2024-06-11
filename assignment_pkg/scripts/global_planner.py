#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Float64

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
		
		# Vars for handling movements in X-Y plane
		self.start_loc = [81590.0, 129520.0]
		self.goal_threshold = 5.0
		self.vel = 10.0

		self.can_move_xy = False

		# Start control loop
		self.control_loop()


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')
		self.weather = rospy.get_param('~weather')
		self.weather_value = rospy.get_param('~weather_value')
		self.goal = np.array(rospy.get_param('~goal', [])).reshape(-1, 2)

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def compute_vel_cmd(self, start, end, speed):
		dir_vec = np.array(end) - np.array(start)
		dist = np.linalg.norm(dir_vec)

		if dist > 0:
			unit_vec = (dir_vec) / dist
		else:
			unit_vec = np.zeros_like(dir_vec)

		return unit_vec * speed


	def update_elevation(self, data):
		self.curr_limit = -data.data
		self.can_move_xy = False if self.curr_limit != self.prev_limit else True


	def ue_to_airsim(self, end, start):
		return np.subtract(end, start) / 100


	def control_loop(self):
		curr_pos = self.airsim.get_drone_position()
		goal_pos = self.ue_to_airsim(self.goal, self.start_loc)

		# Loop while drone is far from goal
		while np.linalg.norm(np.array(curr_pos) - np.array(goal_pos)) > self.goal_threshold:
			if self.can_move_xy:
				vel_cmd = self.compute_vel_cmd(curr_pos, goal_pos, self.vel)
				self.airsim.move_vel(vel_cmd)
			else:
				self.airsim.move_z(self.curr_limit-self.alt_threshold)
				self.prev_limit = self.curr_limit
		
		self.__logger.loginfo("Goal reached!!!")
		self.airsim.land()


def main():
	DroneController('global_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
