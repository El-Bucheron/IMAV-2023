# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
import cv2
import numpy as np
from commande_drone import Drone
from time import sleep

drone = Drone()


try:
    while True:
        print(drone.vehicle.rangefinder.distance)
        try:
            print(drone.camera.detection_aruco_2023())
        except:
            print("Aruco non détecté")
        sleep(1)

finally:
    # Fermeture des fenêtres
    cv2.destroyAllWindows()