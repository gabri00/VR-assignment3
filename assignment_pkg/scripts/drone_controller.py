#!/usr/bin/env python

import rospy
import airsim
from std_msgs.msg import Float64

from Logger import Logger


class DroneController:
	def __init__(self, node_name):
		self._node_name = node_name
		rospy.init_node(self._node_name)

		# Init logger
		self.__logger = Logger(self._node_name)

		self.__logger.loginfo("Node started.")

		# Load parameters
		self.__load_params()
		
		# Initialize AirSim client
		self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)
		self.client.takeoffAsync().join()

		self.__logger.loginfo("Drone ready.")

		# Set weather conditions
		if self.weather:
			self.__set_weather()
			self.__logger.loginfo(f"Set weather to {self.weather} with value {self.weather_value}")

		# Define subscribers
		rospy.Subscriber('elevation', Float64, self.moveDrone_z)


	def __load_params(self):
		self.host = rospy.get_param('~host')
		self.port = rospy.get_param('~port')
		self.weather = rospy.get_param('~weather')
		self.weather_value = rospy.get_param('~weather_value')

		if not self.host or not self.port:
			self.__logger.logerr("Host and port parameters are required.")
			rospy.signal_shutdown("Host and port parameters are required.")


	def __set_weather(self):
		self.client.simEnableWeather(True)

		# Select weather condition
		cond = {
			"Rain": airsim.WeatherParameter.Rain,
			"Roadwetness": airsim.WeatherParameter.Roadwetness,
			"Snow": airsim.WeatherParameter.Snow,
			"RoadSnow": airsim.WeatherParameter.RoadSnow,
			"MapleLeaf": airsim.WeatherParameter.MapleLeaf,
			"RoadLeaf": airsim.WeatherParameter.RoadLeaf,
			"Dust": airsim.WeatherParameter.Dust,
			"Fog": airsim.WeatherParameter.Fog,
		}
		self.client.simSetWeatherParameter(cond[self.weather], self.weather_value)


	def moveDrone_z(self, data):
		self.client.moveToZAsync(-data.data, 1.0)


	def land(self):
		self.client.hoverAsync()
		self.client.landAsync()
	

	def close(self):
		self.client.reset()
		self.client.enableApiControl(False)
	

def main():
	drone_controller = DroneController('drone_controller_node')
	rospy.spin()


if __name__ == '__main__':
	main()
