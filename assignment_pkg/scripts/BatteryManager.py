#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int64
import airsim
import time
import numpy as np


def DistanceCalculator(event):
    global battery_level, speed_kmh, waypoints

    # Calcola la distanza tra i due punti
    initial_distance = calculate_distance(start_lat, start_lon, end_lat, end_lon)

    # Calcola la massima distanza percorribile dalla batteria
    max_distance = max_distance(speed_kmh, battery_level)  # Velocità drone 10 km/h, livello della batteria

    # Controlla se il drone può raggiungere direttamente il punto finale
    if initial_distance <= max_distance:
        rospy.loginfo('Drone diretto al punto finale.') # Il drone continua ad andare al goal settato nel drone_controller
    else:
        rospy.loginfo('Drone va al waypoint massimo raggiungibile.')
        waypoint_index = find_max_reachable_waypoint(waypoints, max_distance)
        target_waypoint = waypoints[waypoint_index]
        target_pose = client.simGetObjectPose(target_waypoint).position
        client.moveToPosition(target_pose.x_val, target_pose.y_val)


def start_point_callback(msg):
    global start_lat, start_lon
    start_lat = msg.x
    start_lon = msg.y

def end_point_callback(msg):
    global end_lat, end_lon
    end_lat = msg.x
    end_lon = msg.y

def calculate_distance(lat1, lon1, lat2, lon2):
    # Raggio medio della Terra in metri
    R = 6371000.0 

    # Converte le coordinate da gradi a radianti
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)

    # Calcola la differenza di latitudine e longitudine
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Calcola la distanza utilizzando la formula di Haversine
    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c

    return distance

def find_max_reachable_waypoint(waypoints, max_distance):
    global initial_position
    # Ottieni le posizioni dei waypoint
    waypoint_positions = []
    for waypoint_name in waypoints:
        object_id = waypoint_name  # Supponendo che l'object ID corrisponda al nome del waypoint
        pose = client.simGetObjectPose(object_id).position
        waypoint_positions.append((pose.x_val, pose.y_val))

    for waypoint_pose in waypoint_positions:
        distance_to_waypoint = np.linalg.norm(np.array(waypoint_pose) - initial_position)  # Calcola la distanza euclidea
        distances.append(distance_to_waypoint)  # Salva la distanza calcolata
      
    # Trova il valore massimo di distanza minore di max_distance
    max_reachable_distance_waypoint_index = enumerate(max([d for d in distances if d < max_distance], default=-1))

    return max_reachable_distance_waypoint_index


def max_distance(speed_kmh, battery_level):
    global speed_kmh, battery_level

    # Conversione della velocità in metri al secondo
    speed_ms = speed_kmh * (1000 / 3600)  # 1 km/h = 1000 m / 3600 s

    # Tempo in secondi
    time_sec = 20

    # Calcolo della distanza percorsa in 20 secondi
    distance_per_20s = speed_ms * time_sec

    # Calcolo della massima distanza percorribile
    max_distance = distance_per_20s * battery_level

    return max_distance


def decrement_battery(event):
    global battery_level

    # Decrementa la batteria di 1 ogni 20 secondi
    battery_level -= 1
    battery_pub.publish(self.battery_level)
    rospy.loginfo('Batteria decrementata. Livello attuale: %d', self.battery_level)


def main():
    global battery_level, speed_kmh, waypoints, initial_position

    rospy.init_node('distance_calculator_node')

    speed_kmh = 10
    battery_level = 100

    waypoints = ['Wp1', 'Wp2', 'Wp3', 'Wp4', 'Wp5', 'Wp6', 'Wp7', 'Wp8']


    # Subscriber per le coordinate di partenza e fine
    rospy.Subscriber('start_point', Point, self.start_point_callback)
    rospy.Subscriber('end_point', Point, self.end_point_callback)

    # Publisher per il livello della batteria
    battery_pub = rospy.Publisher('battery_level', Int64, queue_size=10)

    # Initialize AirSim client
	host = rospy.get_param('~host')
	client = airsim.MultirotorClient(ip=host, port=41451)
	client.confirmConnection()

    # Quad initials
    position = client.getMultirotorState().kinematics_estimated.position
    initial_position = np.array([position.x_val, position.y_val])

	time.sleep(3)

	# Initialize timer to check obstacles distance
	rospy.Timer(rospy.Duration(2), DistanceCalculator)

    # Inizializza un timer che decrementa la batteria ogni 20 secondi
    rospy.Timer(rospy.Duration(20), self.decrement_battery)

    rospy.spin()


if __name__ == '__main__':
	main()