#!/usr/bin/env python

import rospy
import airsim
import geopandas as gpd

def main():
	rospy.init_node("drone_controller_node")
	
	# Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()    
	client.enableApiControl(True)    
	client.armDisarm(True)
	client.takeoffAsync().join()

	try:
		gpd.io.file.fiona.drvsupport.supported_drivers['KML'] = 'rw'
	except:
		pass

	restriction_areas = gpd.read_file('../../flight_map/flight_restriction_areas.kml', driver='KML')

	rospy.spin()

if __name__ == '__main__':
	main()
