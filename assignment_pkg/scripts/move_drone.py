#!/usr/bin/env python

import rospy
import airsim
import time
from Environment import DroneEnvironment

def angle_to_target(position, quaternion):
    angle_to_target = env.calculate_angle(position, quaternion, for_Mline=True)
    return angle_to_target

def bugAlgorithm():


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
