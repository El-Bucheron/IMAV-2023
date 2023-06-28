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

try:
    # Asservissemet du drone pour la positino de l'aruco
    while True:
        drone.asservissement_suivi_vehicule()
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    print("Fin de programme")