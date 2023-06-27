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
drone.lancement_decollage(10)
sleep(1)
#Premier Waypoint
point = LocationGlobalRelative(48.706580, 7.734317, 10)
drone.goto(point, 1)
sleep(5)
#Deuxième Waypoint
point = LocationGlobalRelative(48.706499, 7.734165, 10)
drone.goto(point, 1)
sleep(5)
#Troisième Waypoint
point = LocationGlobalRelative(48.706411, 7.734345, 10)
drone.goto(point, 1)
sleep(5)
#Quatrième Waypoint
point = LocationGlobalRelative(48.706502, 7.734511, 10)
drone.goto(point, 1)
sleep(1)
#Atterissage
drone.set_mode("RTL")