#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import airsim
import time


def check_obstacles(event):

	# Get current Distance data
	distance_sensor_data = client.getDistanceSensorData(distance_sensor_name = '', vehicle_name = '')

	# Publish current obstacle distance
	rospy.loginfo('Distance from obstacle: %f', distance_sensor_data.distance)
	distance_pub.publish(distance_sensor_data.distance)


def main():
	global client, distance_pub

	rospy.init_node('distance_sensor_node')

	# Publishers
	distance_pub = rospy.Publisher('distance_data', Float64, queue_size=10)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()

	time.sleep(3)

	# Initialize timer to check obstacles distance
	rospy.Timer(rospy.Duration(2), check_obstacles)

	rospy.spin()


if __name__ == '__main__':
	main()
