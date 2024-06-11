#!/usr/bin/env python

import rospy
from assignment_pkg.srv import Elevation_srv, Elevation_srvResponse
import requests

from Logger import Logger


class ElevationService:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)
		self.__logger.loginfo("Node started.")

		# Init service
		s = rospy.Service('elevation_srv', Elevation_srv, self.get_elevation)
		self.__logger.loginfo('Service is ready to provide data.')


	def get_elevation(self, req):
		api_url = 'https://api.open-elevation.com/api/v1/lookup?locations='
		url = api_url + str(req.latitude) + ',' + str(req.longitude)

		res_srv = Elevation_srvResponse()
		response = requests.get(url)

		if response.status_code == 200:
			data = response.json()
			if 'results' in data and len(data['results']) > 0:
				res_srv.elevation = float(data['results'][0]['elevation'])
				if res_srv.elevation == -1:
					res_srv.elevation = 0
			else:
				self.__logger.logerr('No results found for the provided coordinates.')
				res_srv.elevation = 0
		else:
			self.__logger.logerr(f'Error occurred while fetching data: {response.status_code}')
			res_srv.elevation = 0

		return res_srv


def main():
	ElevationService('elevation_service')
	rospy.spin()


if __name__ == '__main__':
    main()

