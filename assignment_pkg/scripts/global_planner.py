#!/usr/bin/env python

import numpy as np
import rospy
import math
from std_msgs.msg import Float64
from assignment_pkg.msg import ObstDetected

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
		rospy.Subscriber('/obst_detected', ObstDetected, self.is_obst)

		# Vars for altitude handling
		self.curr_limit = 0.0
		self.prev_limit = -1.0
		self.alt_threshold = 5.0
		
		# Vars for handling movements in X-Y plane
		self.start_loc = np.array([0.0, 0.000364])
		self.goal_threshold = 5.0
		self.can_move_xy = False
		self.vel = 5.0
		self.vel_cmd = np.array([0, 0])
		self.obst_left = False
		self.obst_right = False

		self.goal_pos = self.ue_to_airsim(self.goal, self.start_loc)

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


	def is_obst(self, msg):
		self.obst_left = msg.left
		self.obst_right = msg.right


	def update_elevation(self, data):
		self.curr_limit = -25
		self.can_move_xy = False if self.curr_limit != self.prev_limit else True


	def ue_to_airsim(self, end, start):
		return np.array(np.subtract(end, start) / 100)
	
	
	def get_vec(self, start, end):
		v = end - start
		return v / np.linalg.norm(v)


	def compute_vel(self, curr_pos):
		if not self.obst_left and not self.obst_right:
			goal_vec = self.goal_pos - curr_pos
			return np.array(self.vel * (goal_vec / np.linalg.norm(goal_vec)))
		elif self.obst_left:


	def control_loop(self, event):
		curr_pos = self.airsim.get_drone_position()

		yaw = math.atan2(self.goal_pos[1] - curr_pos[1], self.goal_pos[0] - curr_pos[0]) * 180 / math.pi
		#self.__logger.loginfo(f"YAW: {yaw}")
		#self.__logger.loginfo(f"YAW: {self.airsim.get_yaw()}")

		# Loop while drone is far from goal
		if np.linalg.norm(curr_pos - self.goal_pos) > self.goal_threshold:
			if self.can_move_xy:
				if abs(yaw) - abs(self.airsim.get_yaw()) > 10 and not self.vel_cmd[1]:					
					self.airsim.set_yaw(yaw)
					#self.__logger.loginfo("ADJUSTING YAW...")
				else:
					self.vel_cmd = self.compute_vel(curr_pos)
					self.airsim.move_vel(self.vel_cmd)
					self.__logger.loginfo("MOVING...")
			else:
				self.__logger.loginfo("ELEVATING...")
				self.airsim.move_z(self.curr_limit-self.alt_threshold)
				self.prev_limit = self.curr_limit
		else:
			self.__logger.loginfo("Goal reached!!!")
			self.airsim.land()


def main():
	GlobalPlanner('global_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
