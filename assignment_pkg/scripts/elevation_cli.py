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


def check_gps(event):
	global curr_limit

	# Get current GPS data
	gps_data = client.getGpsData(gps_name = "", vehicle_name = "")
	rospy.loginfo("Lat: %f, Long: %f, Alt: %f" % (gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude))

	# Check if the drone is within a flight restriction area, and if so, get the current altitude limit
	is_in_area = restriction_areas.contains(Point(gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude))
	if is_in_area.any():
		rospy.loginfo("Drone IN restriction area")
		area_limit = int(restriction_areas[is_in_area]['Description'].iloc[0])
	else:
		area_limit = 120
		rospy.loginfo("Drone NOT IN restriction area")

	try:
		elevation_cli = rospy.ServiceProxy("elevation_srv", Elevation_srv)
		resp = elevation_cli(float(gps_data.gnss.geo_point.latitude), float(gps_data.gnss.geo_point.longitude))
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s" % e)

	curr_limit = resp.elevation + area_limit
	rospy.loginfo("Current altitude limit: %d" % curr_limit)

	# Publish current altitude limit
	elevation_pub.publish(curr_limit)


def main():
	rospy.wait_for_service('elevation_srv')
	rospy.init_node("elevation_cli_node")
	global client, restriction_areas, elevation_pub
	
	path_to_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'maps', 'flight_restriction_areas.kml')

	# Load flight restriction areas
	fiona.supported_drivers['KML'] = 'rw'
	restriction_areas = gpd.read_file(path_to_file, driver='KML')

	# Publishers
	elevation_pub = rospy.Publisher("elevation", Float64, queue_size=10)

	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()
	client.enableApiControl(True)
	client.armDisarm(True)
	client.takeoffAsync().join()

	time.sleep(3)

	# Initialize timer to check GPS data
	rospy.Timer(rospy.Duration(1), check_gps)

	rospy.spin()

if __name__ == '__main__':
	main()
