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

drone = Drone()

while True:
    while drone.get_mode() != "GUIDED":
        print("En attente du mode auto")
        sleep(1)
    drone.atterrissage_aruco()