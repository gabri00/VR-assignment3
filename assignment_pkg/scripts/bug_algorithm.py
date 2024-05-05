#!/usr/bin/env python

import rospy
import airsim
import time
from Environment import DroneEnvironment

def angle_to_target(position, quaternion):
    angle_to_target = env.calculate_angle(position, quaternion, for_Mline=True)
    return angle_to_target

def bugAlgorithm():
    current_state = 'align_robot_heading'
    theta_threshold = 4.0 
    threshold_distance = 5.0
    sensor_noise = 0.82 # distance sensor of airsim has a gaussian noise and this value helps in reduce te noise impact on decision

    while True:
        sensor_data = client.getDistanceSensorData(distance_sensor_name = '', vehicle_name = '').distance
        position, quaternion = env.current_position() # Retrieve current position & quaternion (orientation) data
        is_rate, angle_value = True, 0

        # Aligns the heading angle of the quad to the target goal position.
        if current_state == 'align_robot_heading':
            angle_target = angle_to_target(position, quaternion)

            if theta_threshold > min(angle_target, 360-angle_target):
                current_state = 'move_to_goal'
            else: # robot heading is not aligned, thus rotate by yaw
                if angle_target <= 180:
                    action = 4 # rotate right
                else:
                    action = 5 # rotate left
        elif current_state == 'move_to_goal':
            action = 0
            distance = sensor_data.max_distance - sensor_data.min_distance - sensor_noise

            if distance < threshold_distance:
                current_state = 'wall'

        solved = env.step(action, is_rate, angle_value)
        time.sleep(0.25) # delay for correct movement

        if solved:
            return


def main():
    global env, client

    rospy.init_node('bug_node')

    # Initialize AirSim client
    host = rospy.get_param('~host')
    client = airsim.MultirotorClient(ip=host, port=41451)
    client.confirmConnection()

    env = DroneEnvironment(client)
    bugAlgorithm()

    rospy.spin()


if __name__ == '__main__':
    main()
