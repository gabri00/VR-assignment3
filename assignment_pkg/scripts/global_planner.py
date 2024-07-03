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
		#self.airsim.takeoff()
		self.__logger.loginfo("Drone ready.")

		# Set weather conditions
		if self.weather:
			self.airsim.load_weather(self.weather, self.weather_value)
			self.__logger.loginfo(f"Set weather to '{self.weather}' with value '{self.weather_value}'")
		else:
			self.weather_value = 0

		# Define subscribers
		rospy.Subscriber('/elevation_limit', Float64, self.update_elevation)
		rospy.Subscriber('/vel_cmd', VelCmd, self.set_vel)

		# Vars for altitude handling
		self.curr_alt_limit = -25.0
		self.prev_alt_limit = -1.0
		self.alt_threshold = 5.0
		
		# Vars for handling movements in X-Y plane
		self.goal_threshold = 3.0
		self.yaw_threshold = 5.0
		self.can_move_xy = False
		self.vel_cmd = np.array([0, 0])

		self.payload_weight = 4.0 # Weight of the payload in kg
		self.drone_weight = 2.0 # Weight of the drone in kg
		self.autonomy = 100 / (0.05 * (self.payload_weight + self.drone_weight) + self.weather_value) # Autonomy in meters

		self.goal_pos = self.airsim.get_obj_position("StaticMeshActor_3")

		# Get recharge stations position
		self.recharge_objs = ["StaticMeshActor_0", "StaticMeshActor_1", "StaticMeshActor_2"]
		self.recharge_pos = []
		for i in self.recharge_objs:
			self.recharge_pos.append(self.airsim.get_obj_position(i))

		# Start control loop
		rospy.Timer(rospy.Duration(1), self.control_loop)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')
		self.weather = rospy.get_param('~weather')
		self.weather_value = rospy.get_param('~weather_value')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def set_vel(self, msg):
		self.vel_cmd = np.array([msg.vx, msg.vy])


	def update_elevation(self, msg):
		self.curr_alt_limit = -msg.data
		self.can_move_xy = False if self.curr_alt_limit != self.prev_alt_limit else True


	def decide_target(self):
		curr_pos = self.airsim.get_drone_position()
		
		goal_dist = np.linalg.norm(curr_pos - self.goal_pos[:-1])
		# Get dist from drone to each recharge station
		recharge_dist = []
		for i in range(0, 4):
			recharge_dist.append(np.linalg.norm(curr_pos - self.recharge_pos[i][:-1]))

		if goal_dist <= self.autonomy:
			self.__logger.loginfo("Battery sufficient to reach the goal. Heading to the goal.")
			return self.goal_pos
		else:
			# Get the closest recharge station
			target = self.recharge_pos[recharge_dist.index(min(recharge_dist))]
			self.__logger.loginfo("Battery not sufficient to reach the goal. Heading to a recharge station.")
			return target


	def control_loop(self, event):
		self.target_pos = self.decide_target()

		curr_pos = self.airsim.get_drone_position()

		# Loop while drone is far from goal
		if np.linalg.norm(curr_pos - self.target_pos[:-1]) > self.goal_threshold:
			yaw = np.degrees(math.atan2(self.target_pos[1] - curr_pos[1], self.target_pos[0] - curr_pos[0]))
			if self.can_move_xy:
				if abs(yaw) - abs(self.airsim.get_yaw()) > self.yaw_threshold:
					self.__logger.loginfo("Adjusting yaw...")				
					self.airsim.set_yaw(yaw)
				else:
					self.__logger.loginfo("Moving...")

				self.airsim.move_vel(self.vel_cmd)
			else:
				self.__logger.loginfo(f"LIMIT: {self.curr_alt_limit}")
				if self.curr_alt_limit != self.prev_alt_limit:
					self.__logger.loginfo("Elevating...")
					self.airsim.move_z(self.curr_alt_limit+self.alt_threshold)
				self.prev_alt_limit = self.curr_alt_limit
		else:
			self.airsim.move_z(self.target_pos[-1])
			# Check if target is the goal
			if np.array_equal(self.target_pos, self.goal_pos):
				self.__logger.loginfo("Goal reached!!!")
				rospy.signal_shutdown("Goal reached!!!")
			else:
				self.__logger.loginfo("Waypoint reached!!!")
				self.prev_alt_limit = -1.0


def main():
	GlobalPlanner('global_planner')
	rospy.spin()


if __name__ == '__main__':
	main()
