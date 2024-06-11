#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import airsim
import time
import numpy as np

def setup_drone():
    """Setup the AirSim drone client and perform initial actions."""
    client = airsim.MultirotorClient()
    print('Client AirSim creato con successo')
    client.confirmConnection()
    print('Connessione confermata con successo')
    client.enableApiControl(True)
    print('API abilitata con successo')
    client.armDisarm(True)
    print('Drone armato con successo')
    client.takeoffAsync().join()
    print('Decollo completato con successo')
    return client

def get_lidar_data(client):
    """Retrieve and process LiDAR data from the drone."""
    lidarData = client.getLidarData()
    if len(lidarData.point_cloud) < 3:
        print("No points received from Lidar data")
        return None

    points = np.array(lidarData.point_cloud, dtype=np.float32)
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    return points

def calculate_weights(points):
    """Calculate weights for obstacle detection based on LiDAR points."""
    left_weights = []
    right_weights = []
    front_weights = []
    for point in points:
        x, y, z = point
        if x > 0 and abs(z) < 2:
            weight = 1 / (x**2 + y**2 + z**2)
            if y < -1:
                left_weights.append(weight)
            elif y > 1:
                right_weights.append(weight)
            else:
                front_weights.append(weight)

    left_weight_sum = sum(left_weights)
    right_weight_sum = sum(right_weights)
    front_weight_sum = sum(front_weights)
    return left_weight_sum, right_weight_sum, front_weight_sum

def decide_movement(left_weight_sum, right_weight_sum, front_weight_sum):
    """Decide the movement based on the weights of obstacles."""
    turn_weight = right_weight_sum - left_weight_sum
    if front_weight_sum > 0:
        print("Ostacolo rilevato davanti")
    elif turn_weight > 0:
        print("Ostacolo rilevato a destra")
    else:
        print("Ostacolo rilevato a sinistra")

    forward_speed = 3 - (abs(turn_weight) + front_weight_sum)
    forward_speed = max(0, forward_speed)
    print("forward_speed: ", forward_speed)

    return turn_weight, forward_speed

def move_drone(client, vx, vy, vz, duration=1):
    """Move the drone with given velocities."""
    client.moveByVelocityBodyFrameAsync(vx, vy, vz, duration).join()

def avoid_obstacle(client, turn_weight, front_weight_sum):
    """Avoid obstacle by moving the drone laterally."""
    while True:
        if front_weight_sum > 0:
            if turn_weight > 0:
                move_drone(client, 0, -1, 0)
            else:
                move_drone(client, 0, 1, 0)
        else:
            if turn_weight > 0:
                move_drone(client, 0, -1, 0)
            else:
                move_drone(client, 0, 1, 0)

        points = get_lidar_data(client)
        if points is None:
            continue

        left_weight_sum, right_weight_sum, front_weight_sum = calculate_weights(points)
        turn_weight, forward_speed = decide_movement(left_weight_sum, right_weight_sum, front_weight_sum)
        
        if forward_speed > 1:
            break

def main():
    client = setup_drone()

    while True:
        points = get_lidar_data(client)
        if points is None:
            continue

        left_weight_sum, right_weight_sum, front_weight_sum = calculate_weights(points)
        turn_weight, forward_speed = decide_movement(left_weight_sum, right_weight_sum, front_weight_sum)

        if forward_speed < 1:
            avoid_obstacle(client, turn_weight, front_weight_sum)
        else:
            move_drone(client, forward_speed, 0, 0)

        time.sleep(0.1)

    # Atterraggio
    client.landAsync().join()

    # Disarmare il drone
    client.armDisarm(False)

    # Chiudi la connessione
    client.enableApiControl(False)

    print("End...")

if __name__ == "__main__":
    main()
