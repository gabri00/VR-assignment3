#!/usr/bin/env python

import rospy
import airsim
import time


def angle_to_target(position, quaternion):
    angle_to_target = env.calculate_angle(position, quaternion, for_Mline=True)
    return angle_to_target

def bugAlgorithm():
    # Implementazione dell'algoritmo Bug
    prevoius_state = ''
    current_state = 'align_robot_heading'
    theta_threshold = 4 
    sensor_noise = 0.82 # distance sensor of airsim has a gaussian noise and this value helps in reduce te noise impact on decision
    threshold_distance_obstacle = 5.0

    env.client.simGetCollisionInfo().has_collided
    start = time.time()

    while true:
        sensor_data = env.all_distance_sensors() # Retrieve all sensors data
        position, quaternion = env.current_position() # Retrieve current position & quaternion (orientation) data
        is_rate, angle_value = True, 0

        # Aligns the heading angle of the quad to the target goal position.
        if current_state == 'align_robot_heading':
            
            angle_target = angle_to_target(position, quaternion)
            if theta_threshold > min(angle_target, 360-angle_target):
                prevoius_state = current_state
                current_state = 'move_to_goal'
            else: # robot heading is not aligned, thus rotate by yaw
                if angle_target <= 180:
                    action = 4 # rotate right
                else:
                    action = 5 # rotate left

        if current_state == 'move_to_goal':
            action = 0
            front_sensor = env.client.getDistanceSensorData('Distance_0') # oppure .getDistanceSensorData('Distance_0').distance
            distance = front_sensor_data.max_distance - front_sensor_data.min_distance - sensor_noise

            if distance < threshold_distance:
                prevoius_state = current_state
                current_state = 'wall'


    solved = env.step(action, is_rate, angle_value)
    time.sleep(0.25) # delay for correct movement

    if solved:
        # Write log of training
        elapsed_time = abs(round(time.time() - start, 2))
        minutes = int(elapsed_time // 60)
        seconds = int(round(elapsed_time % 60, 2))
        result = (f'Navigation Done. Time: {minutes}:{seconds}\n')
        print(result)
        return


def main():
    global env

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
