"""
Programme permettant de simuler une petit mission dans la zone de Brumath
La mission se déroule de la manière suivante :
    On attend que le drone soit passé en mode Auto (raisons de sécurité)
    Le drone décole à 10 m
    Le drone passe par 4 points de passage formant un carré avec une pause de 5 secondes à chaque point
    Atterissage du drone
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
#Décollage
drone.lancement_decollage(5)
sleep(1)
# On augmente la hauteur du drone d'un mètre à chaque itération et on vérifie que l'aruco est bien détecté
for i in range(20):
    # On définit le prochain point de passage comme étant un mètre plus haut que précédemment
    point = LocationGlobalRelative(drone.vehicle.location.global_relative_frame.lat, 
                                   drone.vehicle.location.global_relative_frame.lon, 
                                   5+i)
    # On fait aller le drone à ce point
    drone.goto(point, 0.1)
    # On récupre les coordonnées du centre de l'aruco
    X, Y = drone.camera.detection_aruco_2023()
    # Si la valeur renvoyé n'est pas nulle, l'aruco a bien été détecté et on affiche l'altitude à laquelle il est détecté 
    if X != None:
        print("Aruco détecté à une altitude de " + drone.vehicle.rangefinder.distance + " mètres")
    # Sinon l'aruco n'a pas été détecté et on affiche l'altitude de non détection
    else:
        print("Aruco non détecté à une altitude de " + drone.vehicle.rangefinder.distance + " mètres")
    # Temporisation d'une seconde 
    sleep(1)

#Atterissage
drone.set_mode("LAND")