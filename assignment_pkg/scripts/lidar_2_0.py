#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import airsim
import time
import numpy as np

# Funzione per ottenere i dati del LIDAR
def get_lidar_data(client):
    lidar_data = client.getLidarData()
    if len(lidar_data.point_cloud) < 3:
        return None
    points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0]/3), 3))
    return points

# Funzione per controllare la presenza di ostacoli
def check_obstacles(points):
    front = points[(points[:,0] > 1) & (points[:,0] < 3)]
    left = points[(points[:,1] > 1) & (points[:,1] < 3)]
    right = points[(points[:,1] < -1) & (points[:,1] > -3)]
    return len(front) > 0, len(left) > 0, len(right) > 0

# Funzione per muovere il drone
def move_drone(client, goal):
    while True:
        lidar_data = get_lidar_data(client)
        if lidar_data is None:
            continue

        front, left, right = check_obstacles(lidar_data)
        
        if front:
            print("Ostacolo rilevato di fronte.")
            if not left and not right:
                print("Ostacoli su entrambi i lati. Fermata in attesa di istruzioni.")
                client.moveByVelocityAsync(0, 0, 0, 1).join()
                continue
            if right and not left:
                print("Spostamento a sinistra.")
                client.moveByVelocityAsync(0, -1, 0, 5).join()
            elif left and not right:
                print("Spostamento a destra.")
                client.moveByVelocityAsync(0, 1, 0, 5).join()
            else:
                
                print("ELSE: Spostamento a sinistra.")
                client.moveByVelocityAsync(0, -1, 0, 5).join()
                
            time.sleep(5)
        else:
            print("Spostamento in avanti.")
            client.moveByVelocityAsync(1, 0, 0, 1).join()

        time.sleep(0.1)

        # Check if the goal is reached
        # position = client.getMultirotorState().kinematics_estimated.position
        # if np.linalg.norm([position.x_val - goal[0], position.y_val - goal[1], position.z_val - goal[2]]) < 1:
        #    print("Goal raggiunto!")
        #    break

# Connessione al client AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Impostazione del goal
# goal = [100, 100, 0]

# Decollo del drone
client.takeoffAsync().join()

# Spostamento del drone verso il goal con obstacle avoidance
move_drone(client, goal)

# Atterraggio del drone
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
