"""
Description:
Code permettant de savoir à partir de quelle altitude on détecte un certain aruco marker

Spécificités:
On attend tout d'abord que le drone soit mis en "GUIDED". 
Une fois que le drone est en mode guided, on décole à 5 mètres de haut, puisque pour un aruco de 20x20 cm, l'aruco est détecte
On itère ensuite la boucle suivante:
    Augmentation de l'altitude du drone d'un mètre
    Annonce de la détection ou non de l'aruco
La boucle se répète jusqu'à ce que le drone ait atteint une altitude de 30 mètres ou qu'un Ctrl+C soit utilisé
"""

#Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from time import sleep
from commande_drone import Drone
from dronekit import LocationGlobalRelative

#Instanciation d'un objet "Drone" pour contrôler le drone 
drone = Drone()
# On attend que le drone soit en mode STABILIZE puis "GUIDED"
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
while drone.get_mode() != "GUIDED":
    print("En attente du mode GUIDED")
# On fait s'envoler le drone à une atltitude de 5 mètres
drone.arm_and_takeoff(5)
# Temporisation d'une seconde 
sleep(1)

try:
    # On augmente la hauteur du drone d'un mètre à chaque itération et on vérifie que l'aruco est bien détecté
    for i in range(20):
        # On définit le prochain point de passage comme étant un mètre plus haut que précédemment
        point = LocationGlobalRelative(drone.vehicle.location.global_relative_frame.lat, 
                                    drone.vehicle.location.global_relative_frame.lon, 
                                    5+i)
        # On fait aller le drone à ce point
        drone.goto(point, 0.1)
        # On récupre les coordonnées du centre de l'aruco
        X, Y = drone.camera.detection_aruco()
        # Si la valeur renvoyé n'est pas nulle, l'aruco a bien été détecté et on affiche l'altitude à laquelle il est détecté 
        if X != None:
            print("Aruco détecté à une altitude de " + drone.vehicle.rangefinder.distance + " mètres")
        # Sinon l'aruco n'a pas été détecté et on affiche l'altitude de non détection
        else:
            print("Aruco non détecté à une altitude de " + drone.vehicle.rangefinder.distance + " mètres")
        # Temporisation d'une seconde 
        sleep(1)
        
# On arrête l'ascencion du drone avec un Ctrl+C
except KeyboardInterrupt:
    #Atterissage
    print("Atterissage")
    drone.set_mode("LAND")