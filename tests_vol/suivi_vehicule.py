"""
Description:
Code peremttant de tester l'asservissement pour le suivi de véhicule

Spécificités:
On attend tout d'abord que le drone soit mis en "GUIDED". 
Pendant ce temps, on affiche que le programme est en attente du mode "GUIDED", l'altitude du drone et si un aruco est détecté
Le code ne prend pas en compte le cas où l'aruco n'est pas détecté. Il faut donc commencer dans une configuration où l'aruco est détecté.
On appelle ensuite la fonction initiant l'asservissement du drone par rapport à l'aruco.
L'asservissement est arrêté par un Ctrl+C
"""

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

# Attente du mode "STABIIZE"
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
    
# Attente du mode "GUIDED"
while drone.get_mode() != "GUIDED":    
    print("En attente du mode GUIDED")

try:
    print("Début de l'assservissement")
    while True:
        centre_aruco_X, centre_aruco_Y, image, image_filtrée = drone.camera.detection_aruco(True)
        drone.asservissement_suivi_vehicule(centre_aruco_X, centre_aruco_Y)
        
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    print("Fin de programme")