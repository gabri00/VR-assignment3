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

        self.goal_pos = self.airsim.get_obj_position("StaticMeshActor_3")
        self.recharge_pos = self.airsim.get_obj_position("StaticMeshActor_0")
        self.goals_pos = np.vstack((self.goal_pos, self.recharge_pos))

        # Battery and payload parameters
        self.battery = 100.0  # Initial battery percentage
        self.payload_weight = 10.0  # Weight of the payload in kg
        self.battery_consumption_rate = 0.1  # Base battery consumption rate per second

        # Weather parameters
        self.weather_impact = 5.0  # Parameter representing the impact of weather conditions
	
	# Decide initial target
        self.target_pos = self.decide_target()
	    
        # Start control loop
        self.tmr = rospy.Timer(rospy.Duration(1), self.control_loop)
        
        # Start battery update loop
        self.battery_tmr = rospy.Timer(rospy.Duration(2), self.update_battery)
        
        

    def __load_params(self):
        self.host = rospy.get_param('~host')
        self.port = rospy.get_param('~port')
        self.weather = rospy.get_param('~weather')
        self.weather_value = rospy.get_param('~weather_value')

        if not self.host or not self.port:
            self.__logger.logerr("Host and port parameters are required.")
            rospy.signal_shutdown("Host and port parameters are required.")

    def calcola_consumo_batteria_per_distanza(self, peso, condizioni_atmosferiche):
        return 0.05 * peso + 0.03 * condizioni_atmosferiche

    def update_battery(self, event):
        # Calculate battery consumption based on payload and weather conditions
        consumo_per_distanza = self.calcola_consumo_batteria_per_distanza(self.payload_weight, self.weather_impact)
        consumption = consumo_per_distanza * 2
        self.battery -= consumption
        self.__logger.loginfo(f"Battery updated: {self.battery}% remaining. Consumption rate: {consumption}% per interval.")
        if self.battery <= 0:
            self.__logger.logerr("Battery depleted. Landing...")
            self.airsim.land()
            rospy.signal_shutdown("Battery depleted.")

    def decide_target(self):
        curr_pos = self.airsim.get_drone_position()
        distanza_goal = self.distance_to_goal(curr_pos, self.goal_pos)
        distanza_rifornimento = self.distance_to_goal(curr_pos, self.recharge_pos)
        
        consumo_per_distanza = self.calcola_consumo_batteria_per_distanza(self.payload_weight, self.weather_impact)
        distanza_massima = (self.battery - 20) / consumo_per_distanza  # 20% is the battery threshold for returning

        if distanza_goal <= distanza_massima:
            self.__logger.loginfo("Battery sufficient to reach the goal. Heading to the goal.")
            return self.goal_pos
        else:
            self.__logger.loginfo("Battery not sufficient to reach the goal. Heading to the recharge station.")
            return self.recharge_pos

    def set_vel(self, msg):
        self.vel_cmd = np.array([msg.vx, msg.vy])

    def update_elevation(self, msg):
        self.curr_alt_limit = -msg.data
        self.can_move_xy = False if self.curr_alt_limit != self.prev_alt_limit else True

    def distance_to_goal(self, pos, goal):
        return np.linalg.norm(pos - goal[:-1])

    def control_loop(self, event):
        curr_pos = self.airsim.get_drone_position()

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
                    self.airsim.move_z(self.curr_alt_limit + self.alt_threshold)
                self.prev_alt_limit = self.curr_alt_limit
        else:
            self.airsim.move_z(self.target_pos[-1])
            if np.array_equal(self.target_pos, self.goal_pos):
                self.__logger.loginfo("Goal reached!!!")
            else:
                self.__logger.loginfo("Recharge station reached. Recharging...")
                self.battery = 100.0  # Reset battery to full after reaching recharge station
                # self.target_pos = self.decide_target()

def main():
    GlobalPlanner('global_planner')
    rospy.spin()

if __name__ == '__main__':
    main()
