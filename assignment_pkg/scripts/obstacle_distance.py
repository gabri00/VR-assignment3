#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import airsim


def check_obstacles(event):
	global curr_limit

	# Get current Distance data
	distance_sensor_data = client.getDistanceSensorData(distance_sensor_name = "", vehicle_name = "")
	obstacle_distance = distance_sensor_data.distance

	# Publish current obstacle distance
	distance_pub.publish(obstacle_distance)
	rospy.loginfo("Distance from obstacle: %f", obstacle_distance)


def main():

	rospy.init_node('distance_sensor_node')

	# Publishers
	distance_pub = rospy.Publisher('distance_data', Float64, queue_size=10)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()

	time.sleep(3)

	# Initialize timer to check obstacles distance
	rospy.Timer(rospy.Duration(1), check_obstacles)

	rospy.spin()


if __name__ == '__main__':
	main()