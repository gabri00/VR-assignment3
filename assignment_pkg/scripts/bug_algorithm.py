#!/usr/bin/env python

import rospy
import airsim
import time
import numpy as np
from Environment import DroneEnvironment


class Bug2:
    def __init__(self):
        # Initialize AirSim client
        self.host = rospy.get_param('~host')
        self.client = airsim.MultirotorClient(ip=self.host, port=41451)
        self.client.confirmConnection()
        
        self.env = DroneEnvironment(self.client)

        self.m_line = self.env.goal - self.env.initial_position
        self.m_line = self.m_line / np.linalg.norm(self.m_line)

    
    def angle_to_target(self, position, quaternion):
        return self.env.calculate_angle(position, quaternion, for_Mline=True)


    def dist_to_Mline(self, quad_position):
        quad_to_init_vector = quad_position - self.env.initial_position

        # Vector from quad position to M-line (Projection vector of the quad's position onto the M-line.)
        quad_to_mline = np.dot(quad_to_init_vector, self.m_line) * self.m_line

        # Distance to M-line
        return np.linalg.norm(quad_position - (self.env.initial_position + quad_to_mline))


    def run(self):
        prev_state = 'start'
        curr_state = 'align_robot_heading'

        theta_threshold = 4.0 
        dist_threshold = 5.0
        dist_to_Mline_threshold = 0.3
        
        sensor_noise = 0.82 # distance sensor of airsim has a gaussian noise and this value helps in reduce te noise impact on decision

        while True:
            sensor_data = self.client.getDistanceSensorData(distance_sensor_name = '', vehicle_name = '').distance
            position, quaternion = self.env.current_position() # Retrieve current position & quaternion (orientation) data
            is_rate, angle_value = True, 0

            # Align drone to target
            if curr_state == 'align_robot_heading':
                angle_target = self.angle_to_target(position, quaternion)

                if theta_threshold > min(angle_target, 360-angle_target):
                    prev_state = curr_state
                    curr_state = 'move_to_goal'
                else: # robot heading is not aligned, thus rotate by yaw
                    if angle_target <= 180:
                        action = 4 # rotate right
                    else:
                        action = 5 # rotate left
            elif curr_state == 'move_to_goal':
                action = 0

            elif curr_state == 'turn':
                is_rate, angle_value = False, 90

            solved = self.env.step(action, is_rate, angle_value)
            time.sleep(0.25) # delay for correct movement

            if solved:
                return


def main():
    rospy.init_node('bug_node')

    bug = Bug2()
    bug.run()

    rospy.spin()


if __name__ == '__main__':
    main()
