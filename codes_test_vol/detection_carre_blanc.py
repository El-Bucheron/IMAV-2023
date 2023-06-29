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


try:
    while True:
        X,_ = drone.camera.detection_carre_blanc(drone.vehicle.rangefinder.distance)
        print(("Aruco détecté" if X != None else "Aruco non détecté"))
        sleep(1)

except KeyboardInterrupt:
    cv2.destroyAllWindows()
