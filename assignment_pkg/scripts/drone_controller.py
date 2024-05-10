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
		self.__init_client()
		self.__logger.loginfo("Drone ready.")

		# Set weather conditions
		if self.weather:
			self.__set_weather()

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


	def __init_client(self):
		self.client = airsim.MultirotorClient(ip=self.host, port=self.port)
		self.client.confirmConnection()
		self.client.enableApiControl(True)
		self.client.armDisarm(True)
		self.client.takeoffAsync().join()


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

		try:
			self.client.simSetWeatherParameter(cond[self.weather], self.weather_value)
		except KeyError:
			self.__logger.logerr(f"Invalid weather condition: {self.weather}")
			rospy.signal_shutdown(f"Invalid weather condition: {self.weather}")

		self.__logger.loginfo(f"Set weather to {self.weather} with value {self.weather_value}")


	def moveDrone_z(self, data):
		self.client.moveToZAsync(-data.data, 5.0).join()


	def moveToPosition(self, lat, lon, alt):
		self.client.moveToGPSAsync(lat, lon, -alt, 10.0).join()


	def land(self):
		self.client.hoverAsync()
		self.client.landAsync()
	

	def close(self):
		self.client.reset()
		self.client.enableApiControl(False)
	

def main():
	drone_controller = DroneController('drone_controller_node')
	drone_controller.moveToPosition(44.404023, 8.945463, 50)
	rospy.spin()


if __name__ == '__main__':
	main()
