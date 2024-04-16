#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from assignment_pkg.srv import Elevation_srv, Elevation_srvResponse
import airsim
import geopandas as gpd
import fiona
from shapely.geometry import Point


def check_gps(event):
	global curr_limit

	# Get current GPS data
	gps_data = client.getGpsData(gps_name = "", vehicle_name = "")
	rospy.loginfo("Current GPS data: Latitude: %f, Longitude: %f, Altitude: %f" % (gps_data.latitude, gps_data.longitude, gps_data.altitude))

	# Check if the drone is within a flight restriction area, and if so, get the current altitude limit
	is_in_area = restriction_areas.contains(Point(gps_data.latitude, gps_data.longitude))
	if is_in_area.any():
		rospy.loginfo("Drone is within a flight restriction area.")
		area_limit = int(restriction_areas[is_in_area]['Description'].iloc[0])
	else:
		area_limit = 120
		rospy.loginfo("Drone is not within a flight restriction area. Current altitude limit: %d" % curr_limit)

	try:
		elevation_req = rospy.ServiceProxy("elevation_srv", Elevation_srv)
		resp = elevation_req(gps_data.latitude, gps_data.longitude)
		curr_limit = resp.elevation + area_limit
		rospy.loginfo("Current altitude limit: %d" % curr_limit)
		# Publish current altitude limit
		elevation_pub.publish(curr_limit)
	except rospy.ServiceException as e:
		rospy.logerr("Service call failed: %s" % e)


def main():
	rospy.init_node("drone_controller_node")
	global client, restriction_areas, elevation_pub

	# Load flight restriction areas
	fiona.supported_drivers['KML'] = 'rw'
	restriction_areas = gpd.read_file('../../maps/flight_restriction_areas.kml', driver='KML')

	# Publishers
	elevation_pub = rospy.Publisher("elevation", Float32, queue_size=10)

	# Initialize timer to check GPS data
	rospy.Timer(rospy.Duration(1), check_gps)

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
