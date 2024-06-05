#!/usr/bin/env python

# from __future__ import print_function
import airsim
import time

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
    
    def land(self):
        self.client.landAsync().join()
    
    def get_drone_position(self):
        pose = self.client.simGetVehiclePose()
        return [pose.position.x_val, pose.position.y_val]

    def fly_to(self, point):
        curr_z = self.client.simGetVehiclePose().position.z_val
        self.client.moveToPositionAsync(point[0], point[1], curr_z, 5).join()
        
        time.sleep(2)

    def fly_path(self, points):
        airsim_points = []
        for point in points:
            if point[2] > 0:
                airsim_points.append(airsim.Vector3r(point[0], point[1], -point[2]))
            else:
                airsim_points.append(airsim.Vector3r(point[0], point[1], point[2]))
        self.client.moveOnPathAsync(airsim_points, 5, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 20, 1).join()

    def set_yaw(self, yaw):
        self.client.rotateToYawAsync(yaw).join()
        time.sleep(1)

    def get_yaw(self):
        orientation_quat = self.client.simGetVehiclePose().orientation
        yaw = airsim.to_eularian_angles(orientation_quat)[2]
        return yaw

    # def get_position(self, object_name):
    #     query_string = objects_dict[object_name] + ".*"
    #     object_names_ue = []
    #     while len(object_names_ue) == 0:
    #         object_names_ue = self.client.simListSceneObjects(query_string)
    #     pose = self.client.simGetObjectPose(object_names_ue[0])
    #     return [pose.position.x_val, pose.position.y_val, pose.position.z_val]
    def get_obj_position(self, obj_name):
        pose = self.client.simGetObjectPose(obj_name)
        print(f'{self.client.simListSceneObjects()}')
        return [pose.position.x_val, pose.position.y_val]
 

    def get_distance_reading(self, sensor_name):
        return self.client.getDistanceSensorData(sensor_name).distance

    def get_gps_data(self):
        return self.client.getGpsData(gps_name = '', vehicle_name = '')

    def move_z(self, pos):
        self.client.moveToZAsync(pos, 2)
        time.sleep(2)

