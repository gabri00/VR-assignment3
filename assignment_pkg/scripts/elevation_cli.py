#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
from assignment_pkg.srv import Elevation_srv, Elevation_srvRequest, Elevation_srvResponse
import geopandas as gpd
import fiona
from shapely.geometry import Point
import os
import time

from AirSimWrapper import AirSimWrapper
from Logger import Logger


class ElevationClient:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()

		# Initialize AirSim wrapper
		self.airsim = AirSimWrapper(self.host, self.port)

		# Wait for elevation service
		rospy.wait_for_service('elevation_srv')
		self.__logger.loginfo("Elevation service ready.")

		# Publishers
		self.elevation_pub = rospy.Publisher('/elevation_limit', Float64, queue_size=1)

		time.sleep(1)

		# Initialize timer to check GPS data
		rospy.Timer(rospy.Duration(5), self.check_gps)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def load_kml(self, filename):
		path_to_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'maps', filename)
		fiona.supported_drivers['KML'] = 'rw'
		self.restriction_areas = gpd.read_file(path_to_file, driver='KML')


	def check_gps(self, event):
		# Get current GPS data
		gps_data = self.airsim.get_gps_data()
		#self.__logger.loginfo(f'Current altitude: {abs(gps_data.gnss.geo_point.altitude)}')

		# Check if the drone is within a flight restriction area, and if so, get the current altitude limit
		is_in_area = self.restriction_areas.contains(Point(gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.latitude))
		if is_in_area.any():
			#self.__logger.logwarn('Drone IN restriction area')
			area_limit = int(self.restriction_areas[is_in_area]['Description'].iloc[0])
		else:
			area_limit = 120

		try:
			elevation_cli = rospy.ServiceProxy('elevation_srv', Elevation_srv)
			resp = elevation_cli(float(gps_data.gnss.geo_point.latitude), float(gps_data.gnss.geo_point.longitude))
		except rospy.ServiceException as e:
			self.__logger.logerr(f'Service call failed: {e}')

		if resp.elevation == -1:
			self.__logger.logerr('Service error')

		curr_limit = resp.elevation + area_limit
		curr_limit = 25
		#self.__logger.loginfo(f'Max elevation: {curr_limit}')

		# Publish current altitude limit
		self.elevation_pub.publish(curr_limit)


def main():
	elevation_cli = ElevationClient('elevation_cli')
	elevation_cli.load_kml('flight_restriction_areas.kml')

	rospy.spin()


if __name__ == '__main__':
	main()
