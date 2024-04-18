#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from assignment_pkg.srv import Elevation_srv, Elevation_srvRequest, Elevation_srvResponse
import airsim
import geopandas as gpd
import fiona
from shapely.geometry import Point
import os
import time


def moveDrone_z(data):
	client.moveToZAsync(-data.data, 1.0)


def main():
	rospy.init_node("drone_controller_node")
	global client
	
	rospy.Subscriber('elevation', Float64, moveDrone_z)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()
	client.enableApiControl(True)
	client.armDisarm(True)
	client.takeoffAsync().join()

	rospy.spin()

if __name__ == '__main__':
	main()
