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

# Attente du mode "STABILIZE"
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
    
# Attente du mode "GUIDED"
while drone.get_mode() != "GUIDED":    
    print("En attente du mode GUIDED")
    
# Décollage du drone
print("Décollage")
drone.takeoff(15)
sleep(10)

# Début de la manoeuvre d'atterissage
try:
    drone.atterrissage_aruco()  
except KeyboardInterrupt:
    print("Fin de programme")