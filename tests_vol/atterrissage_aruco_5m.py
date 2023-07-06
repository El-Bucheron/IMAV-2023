# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from commande_drone import Drone
from time import sleep

#Instanciation de l'objet drone
drone = Drone()
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
# Attente du mode "GUIDED"
while drone.get_mode() != "GUIDED":    
    # On affiche l'altitude de vol
    print("Altitude = " + str(drone.vehicle.rangefinder.distance))
    # On essaye de détecter un aruco
    X,Y = drone.camera.detection_aruco_2023()
    # Si un aruco est détecté, on affiche les coordonnées renvoyées
    if X != None:
        print("Coordonnées trouvées : x = " + str(X) + " ; y = " + str(Y))
    # Sinon on affiche que l'aruco n'a pas été détecté
    else:
        print("Aruco non détecté")
    sleep(1)
# Début de la manoeuvre d'atterissage
drone.atterrissage_aruco()