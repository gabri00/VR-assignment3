#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int64
import airsim
import time

# Define the meaning of sensor codes
# Code = 1: front sensor 
# Code = 2: right sensor
# Code = 3: left sensor

def check_obstacles(event):
    global code
	
    if code == 1:
        distance_sensor_data = client.getDistanceSensorData('Distance_0')
    elif code == 2:
        distance_sensor_data = client.getDistanceSensorData('Distance_M90')
    elif code == 3:
        distance_sensor_data = client.getDistanceSensorData('Distance_90')
    else:
        rospy.logwarn("Unknown sensor code: %d", code)
        return


	# Get current Distance data
	distance_condition = distance_sensor_data.max_distance - distance_sensor_data.min_distance - sensor_noise
	
	# Publish current obstacle distance
	rospy.loginfo('Distance from obstacle: %f', distance_sensor_data.distance)
	distance_pub.publish(distance_sensor_data.distance)


def sensor_code_callback(msg):
    global code
	
    rospy.loginfo('Sensor code received: %f', msg.data)
    code = msg.data

def main():
	global client, distance_pub

	rospy.init_node('distance_sensor_node')

	# Initialize sensor noise
    sensor_noise = 0.82

	# Publishers
	distance_pub = rospy.Publisher('distance_data', Float64, queue_size=10)

	# Subscribers
    rospy.Subscriber('sensor_code', Int64, sensor_code_callback)

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
