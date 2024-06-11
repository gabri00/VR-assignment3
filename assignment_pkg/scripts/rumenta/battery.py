#!/usr/bin/env python

import math
import time

import numpy as np
import rospy
from std_msgs.msg import Float64
from assignment_pkg.msg import DistData

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
        rospy.Subscriber('/sensor_data', DistData, self.get_sensor_data)

        # Vars for altitude handling
        self.curr_limit = 0.0
        self.prev_limit = -1.0
        self.alt_threshold = 5.0

        # Vars for obstacle avoidance
        self.obst_threshold = 10.0
        self.yaw_step = 45
        
        # Vars for handling movements in X-Y plane
        self.start_loc = [80040.0, 135390.0]
        self.goal_threshold = 5.0
        self.step_size = 5.0
        
        self.can_move_xy = False
        self.sensor_data = None

        # Initialize battery parameters
        self.battery_level = 100.0
        self.battery_decrement_rate = self.calculate_battery_decrement_rate()
        self.battery_threshold = 10.0  # Critical battery level threshold

        # Timing for battery consumption
        self.last_battery_update_time = time.time()

        # Start control loop
        self.control_loop()

    def __load_params(self):
        self.host = rospy.get_param('~host')
        self.port = rospy.get_param('~port')
        self.weather = rospy.get_param('~weather')
        self.weather_value = rospy.get_param('~weather_value')
        self.payload_weight = rospy.get_param('~payload_weight', 0.0)  # Weight of the payload

        if not self.host or not self.port:
            self.__logger.logerr("Host and port parameters are required.")
            rospy.signal_shutdown("Host and port parameters are required.")

    def calculate_battery_decrement_rate(self):
        # Base consumption rate
        base_rate = 0.1
        
        # Increase rate based on weather conditions
        if self.weather == 'rain':
            weather_rate = 0.2
        elif self.weather == 'sunny':
            weather_rate = 0.1
        else:
            weather_rate = 0.1

        # Increase rate based on payload weight
        weight_rate = 0.05 * self.payload_weight

        return base_rate + weather_rate + weight_rate

    def update_battery(self):
        current_time = time.time()
        if current_time - self.last_battery_update_time >= 2:  # Check if 2 seconds have passed
            self.last_battery_update_time = current_time

            # Update battery level based on rate
            if self.battery_decrement_rate < 0.2:
                decrement = 1
            elif self.battery_decrement_rate < 0.3:
                decrement = 2
            else:
                decrement = 3

            self.battery_level -= decrement

            self.__logger.loginfo(f"Battery level: {self.battery_level:.2f}%")
            if self.battery_level <= self.battery_threshold:
                self.__logger.logwarn("Battery critically low! Initiating landing procedure.")
                self.airsim.land()
                rospy.signal_shutdown("Battery critically low")

    def update_elevation(self, data):
        self.curr_limit = -data.data
        self.can_move_xy = False if self.curr_limit != self.prev_limit else True

    def get_sensor_data(self, data):
        self.__logger.loginfo(f"Distance data: {data.front}, {data.right}, {data.left}")
        self.sensor_data = data

    def control_loop(self):
        curr_pos = self.airsim.get_drone_position()
        goal_pos = np.subtract([77190.0, 128099.999999], self.start_loc) / 100

        # Loop while drone is far from goal
        while np.linalg.norm(np.array(curr_pos) - np.array(goal_pos)) > self.goal_threshold:
            self.update_battery()
            if self.can_move_xy:
                # Calculate yaw angle to face the goal
                yaw = math.atan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0]) * 180 / math.pi
                
                # If drone is near an obstacle, turn
                if self.sensor_data.front < self.obst_threshold:
                    turn_sign = -1 if self.sensor_data.right <= self.sensor_data.left else 1
                    yaw += turn_sign * self.yaw_step

                    self.__logger.logwarn("Obstacle detected, turning...")
                
                self.airsim.set_yaw(yaw)

                # Take a step towards the goal while maintaining the yaw angle
                # curr_yaw = self.airsim.get_yaw()
                step_vec = self.step_size * [math.cos(math.radians(yaw)), math.sin(math.radians(yaw))]
                self.__logger.loginfo(f"Step vector: {step_vec}")
                self.airsim.fly_to(np.add(curr_pos, step_vec))

                # Face the goal again by calculating the new yaw angle
                curr_pos = self.airsim.get_drone_position()
            else:
                self.airsim.move_z(self.curr_limit - self.alt_threshold)
                self.prev_limit = self.curr_limit
                self.__logger.logwarn("Changing altitude...")
        
        self.__logger.loginfo("Goal reached!!!")


def main():
    DroneController('controller')
    rospy.spin()


if __name__ == '__main__':
    main()
