# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from time import sleep
from commande_drone import Drone
from utilities import *

drone = Drone()

print("Début de programme")
# Structure "try" pour pouvoir arrêter le programme avec un Ctrl+C
try:
    while True:
        # Détection d'un aruco
        centre_aruco_X, centre_aruco_Y, aruco_id, image = drone.camera.detection_aruco(True)
        # Temporisation 
        sleep(1)        
        # Estimating marker location from vision
        if centre_aruco_X == None:
            print("Aruco non détecté")
            continue
        #Calcul des coordonnées GPS
        estimated_location = get_GPS_through_picture(drone, centre_aruco_X, centre_aruco_Y)
        # Affichage des coordonnées GPS
        print(f"Coordonnées calculées : latitude = {estimated_location.lat:.8f} ; longitude = {estimated_location.lon:.8f}")

except KeyboardInterrupt:
    print("Fin de programme")