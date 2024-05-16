#!/usr/bin/env python

from __future__ import print_function
import rospy
import airsim
import pymap3d as pm
from std_msgs.msg import Float64, Bool
from assignment_pkg.msg import DistData
from airsim_ros_pkgs.msg import GPSYaw
from nav_msgs.msg import Odometry
from airsim_ros_pkgs.srv import SetGPSPosition
import time

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
		# rospy.Subscriber('elevation', Float64, self.moveDrone_z)
		# rospy.Subscriber('ack_move', Bool, self.ack_callback)
		# rospy.Subscriber('distance_data', DistData, self.dist_sens_callback)
		self.home_pub = rospy.Publisher('/airsim_node/home_geo_point', GPSYaw, queue_size=10)
		self.odom_pub = rospy.Publisher('/airsim_node/Drone/odom_local_ned', Odometry, queue_size=10)

		time.sleep(1)
		self.set_home_geo()

		# Init variables
		self.curr_alt = 0.0
		self.can_move = False
		self.obst_thresh = 5.0
		self.state = 'drone_ready'
		
		time.sleep(1)
		self.set_goal()

		rospy.Timer(rospy.Duration(1), self.set_odom)


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


	def set_home_geo(self):
		msg = GPSYaw()
		msg.latitude = self.client.getHomeGeoPoint().latitude
		msg.longitude = self.client.getHomeGeoPoint().longitude
		msg.altitude = self.client.getHomeGeoPoint().altitude
		msg.yaw = 0.0
		self.home_pub.publish(msg)


	def set_odom(self, event):
		gps_data = self.client.getGpsData(gps_name = '', vehicle_name = 'Drone')
		self.curr_pos = pm.geodetic2ned(gps_data.gnss.geo_point.latitude, gps_data.gnss.geo_point.longitude, gps_data.gnss.geo_point.altitude, self.client.getHomeGeoPoint().latitude, self.client.getHomeGeoPoint().longitude, self.client.getHomeGeoPoint().altitude)
		self.__logger.loginfo(f"Current NED position: {self.curr_pos[0]}, {self.curr_pos[1]}, {self.curr_pos[2]}")
		msg = Odometry()
		msg.pose.pose.position.x = self.curr_pos[0]
		msg.pose.pose.position.y = self.curr_pos[1]
		msg.pose.pose.position.z = self.curr_pos[2]
		msg.pose.pose.orientation.w = 1.0
		self.odom_pub.publish(msg)


	def set_goal(self):
		rospy.wait_for_service("/airsim_node/gps_goal")
		try:
			add_two_ints = rospy.ServiceProxy("/airsim_node/gps_goal", SetGPSPosition)
			resp1 = add_two_ints(44.6, 8.9, 2.0, 0.0, "Drone")
			return resp1.success
		except rospy.ServiceException as e:
			self.__logger.logerr(f"Service call failed: {e}")


	def ack_callback(self, data):
		self.can_move = data.data


	def moveDrone_z(self, data):
		self.curr_alt = -data.data
		self.client.moveToZAsync(-data.data, 1.5).join()

		if self.can_move:
			self.moveToPosition(44.404023, 8.945463)


	def moveToPosition(self, lat, lon):
		self.client.moveToGPSAsync(lat, lon, self.curr_alt, 5.0)


	def land(self):
		self.client.hoverAsync()
		self.client.landAsync()
	

	def close(self):
		self.client.reset()
		self.client.enableApiControl(False)


	def dist_sens_callback(self, data):
		self.__logger.loginfo(f"Distance sensors: front={data.data_front}, left={data.data_left}, right={data.data_right}")
		
		if data.data_front < self.obst_thresh:
			self.state = 'obst_front'
		if data.data_left < self.obst_thresh:
			self.state = 'obst_left'
		if data.data_right < self.obst_thresh:
			self.state = 'obst_right'


def main():
	drone_controller = DroneController('drone_controller_node')
	rospy.spin()


if __name__ == '__main__':
	main()
