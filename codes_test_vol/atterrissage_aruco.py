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
        print("En attente du mode GUIDED")
        print("Altitude = " + altitude = drone.vehicle.rangefinder.distance + " m")
        centre_aruco_x, centre_aruco_y = drone.camera.detection_aruco_2023()
        if centre_aruco_x != None:
            print("Aruco trouvé de coordonnées (en pixel): x = " + str(centre_aruco_x) + " ; y = " + str(centre_aruco_y))
        sleep(1)    
    drone.atterrissage_aruco()