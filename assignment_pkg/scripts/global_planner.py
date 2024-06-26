#!/usr/bin/env python

import numpy as np
import rospy
import math
from std_msgs.msg import Float64
from assignment_pkg.msg import VelCmd

from AirSimWrapper import AirSimWrapper
from Logger import Logger


class GlobalPlanner:
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
		rospy.Subscriber('/vel_cmd', VelCmd, self.set_vel)

		# Vars for altitude handling
		self.curr_limit = 0.0
		self.prev_limit = -1.0
		self.alt_threshold = 5.0
		
		# Vars for handling movements in X-Y plane
		self.start_loc = np.array([0.0, 0.000364])
		self.goal_threshold = 5.0
		self.can_move_xy = False
		self.vel_cmd = np.array([0, 0])

		self.goal_pos = self.ue_to_airsim(self.goal, self.start_loc)
		self.goal_vec = self.get_vec(self.start_loc, self.goal_pos)

		# Start control loop
		rospy.Timer(rospy.Duration(0.5), self.control_loop)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')
		self.weather = rospy.get_param('~weather')
		self.weather_value = rospy.get_param('~weather_value')
		goal_x = rospy.get_param('~goal_x')
		goal_y = rospy.get_param('~goal_y')
		self.goal = np.array([goal_x, goal_y])

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def set_vel(self, msg):
		self.vel_cmd = np.array([msg.vx, msg.vy])


	def update_elevation(self, data):
		self.curr_limit = -25
		self.can_move_xy = False if self.curr_limit != self.prev_limit else True


	def ue_to_airsim(self, end, start):
		return np.array(np.subtract(end, start) / 100)
	
	
	def get_vec(self, start, end):
		v = end - start
		return v / np.linalg.norm(v)


	def control_loop(self, event):
		curr_pos = self.airsim.get_drone_position()
		curr_vec = self.get_vec(curr_pos, self.goal_pos)

		# Loop while drone is far from goal
		if np.linalg.norm(curr_pos - self.goal_pos) > self.goal_threshold:
			if self.can_move_xy:
				if np.cross(curr_vec, goal_vec) and not vel_cmd[1]:
					yaw = math.atan2(self.goal_pos[1] - curr_pos[1], self.goal_pos[0] - curr_pos[0]) * 180 / math.pi
					self.airsim.set_yaw(yaw)
				else:
					self.airsim.move_vel(vel_cmd)
			else:
				self.airsim.move_z(self.curr_limit-self.alt_threshold)
				self.prev_limit = self.curr_limit
		else:
			self.__logger.loginfo("Goal reached!!!")
			self.airsim.move_vel(np.array([0, 0]))
			self.airsim.land()


def main():
	GlobalPlanner('global_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
