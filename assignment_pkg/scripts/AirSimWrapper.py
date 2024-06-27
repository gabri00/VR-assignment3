#!/usr/bin/env python

import airsim
import time
import numpy as np
import math

class AirSimWrapper:
    def __init__(self, host, port):
        self.client = airsim.MultirotorClient(ip=host, port=port)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

    def load_weather(self, weather, weather_value):
        self.client.simEnableWeather(True)
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
        self.client.simSetWeatherParameter(cond[weather], weather_value)

    def takeoff(self):
        self.client.takeoffAsync().join()
        time.sleep(1)
    
    def land(self):
        self.client.landAsync()
        time.sleep(5)
    
    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return np.array([pose.position.x_val, pose.position.y_val])

    def move_vel(self, vel):
        self.client.moveByVelocityBodyFrameAsync(vel[0], vel[1], 0, 1)

    def set_yaw(self, yaw):
        self.client.rotateToYawAsync(yaw).join()
        time.sleep(1)

    def get_yaw(self):
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2]
        return yaw * 180/math.pi

    def get_lidar_reading(self, sensor_name):
        points = self.client.getLidarData(sensor_name).point_cloud
        rng = round(len(points)/3)
        sensor_data = None
        sensor_data.front = min(points[rng:2*rng])
        sensor_data.left = min(points[0:rng])
        sensor_data.right = min(points[2*rng:(len(points)-1)])
        return sensor_data

    def get_gps_data(self):
        return self.client.getGpsData(gps_name = '', vehicle_name = '')

    def move_z(self, pos):
        self.client.moveToZAsync(pos, 2)
        time.sleep(2)

