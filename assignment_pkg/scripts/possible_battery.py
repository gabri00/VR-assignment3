#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import airsim
import numpy as np

# Funzione per calcolare il consumo della batteria per unità di distanza
def calcola_consumo_batteria_per_distanza(peso, condizioni_atmosferiche):
    return 0.05 * peso + 0.03 * condizioni_atmosferiche

# Parametri
peso_oggetto = 10  # peso dell'oggetto trasportato
condizioni_atmosferiche = 5  # parametro relativo alle condizioni atmosferiche (es. vento, pioggia)
batteria_iniziale = 100  # Capacità iniziale della batteria
soglia_batteria = 20  # Soglia della batteria per il ritorno
threshold = 5  # Soglia di distanza per l'atterraggio

# Connessione al client AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Decollo del drone
client.takeoffAsync().join()
client.moveToPositionAsync(0, 0, -15, 5).join()

# Definizione dei punti
goal = np.array([0, -80])
punto_rifornimento = np.array([0, -40])  # Punto di rifornimento definito

# Calcola il consumo della batteria per unità di distanza
consumo_per_distanza = calcola_consumo_batteria_per_distanza(peso_oggetto, condizioni_atmosferiche)

# Calcola la distanza massima percorribile con la batteria attuale
distanza_massima = (batteria_iniziale - soglia_batteria) / consumo_per_distanza

# Calcola le distanze ai vari punti
pose = client.simGetVehiclePose()
current_position = np.array([pose.position.x_val, pose.position.y_val])
distanza_goal = np.linalg.norm(goal - current_position)
#distanza_rifornimento = np.linalg.norm(punto_rifornimento - current_position)

print(f"Distanza massima percorribile con la batteria attuale: {distanza_massima:.2f} metri")
print(f"Distanza al goal: {distanza_goal:.2f} metri")
#print(f"Distanza al punto di rifornimento: {distanza_rifornimento:.2f} metri")

# Decidi se dirigersi verso il goal o il punto di rifornimento
if distanza_goal <= distanza_massima:
    print("Batteria sufficiente per raggiungere il goal")
    # GO TO GOAL
    client.moveToPositionAsync(float(goal[0]), float(goal[1]), -15, 5).join()
else:
    print("Batteria NON sufficiente per raggiungere il goal")
    # GO TO punto rifornimento
    client.moveToPositionAsync(float(punto_rifornimento[0]), float(punto_rifornimento[1]), -15, 5).join()

# Attendi 2 secondi
#time.sleep(5)

# Atterraggio del drone
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
