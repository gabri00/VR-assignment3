#!/usr/bin/env python

import rospy
import airsim
import sys
import os
import time

def move_drone():
	# Initialize node
	rospy.init_node('test', anonymous=True)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()    
	client.enableApiControl(True)    
	client.armDisarm(True)
	client.takeoffAsync().join()

	print("Moving...")
	client.moveToZAsync(-2185.394453, 1.0)
	time.sleep(5)
	print("End...")
		
	# Land the drone
	client.hoverAsync()
	client.landAsync()

	# Chiudi la connessione al client di AirSim
	client.reset()
	client.enableApiControl(False)

if __name__ == '__main__':
    try:
        move_drone()
    except rospy.ROSInterruptException:
        pass
