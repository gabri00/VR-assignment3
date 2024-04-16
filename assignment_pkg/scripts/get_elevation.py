#!/usr/bin/env python

import rospy
from assignment_pkg.srv import Elevation_srv, Elevation_srvResponse
import requests


def get_elevation(req):
	api_url = 'https://api.open-elevation.com/api/v1/lookup?locations='
	url = api_url + (str(req.latitude) + ',' + str(req.longitude))

	res_srv = Elevation_srvResponse()
	response = requests.get(url)

	if response.status_code == 200:
		data = response.json()
		if 'results' in data and len(data['results']) > 0:
			res_srv.elevation = data['results']['elevation']
		else:
			rospy.logerr("No results found for the provided coordinates.")
			res_srv.elevation = -1
	else:
		rospy.logerr("Error occurred while fetching data:", response.status_code)
		res_srv.elevation = -1


def main():
	rospy.init_node("get_elevation_node")
	rospy.Service("elevation_srv", Elevation_srv, get_elevation)
	rospy.loginfo("Service is ready to provide elevation data.")
	rospy.spin()


if __name__ == "__main__":
    main()

