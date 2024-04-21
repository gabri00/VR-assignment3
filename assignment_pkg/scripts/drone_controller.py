#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import airsim


def moveDrone_z(data):
	client.moveToZAsync(-data.data, 1.0)


def main():
	rospy.init_node('drone_controller_node')
	global client
	
	rospy.Subscriber('elevation', Float64, moveDrone_z)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()
	client.enableApiControl(True)
	client.armDisarm(True)
	client.takeoffAsync().join()

	# # Land the drone
	# client.hoverAsync()
	# client.landAsync()

	# # Chiudi la connessione al client di AirSim
	# client.reset()
	# client.enableApiControl(False)

	rospy.spin()


if __name__ == '__main__':
	main()
