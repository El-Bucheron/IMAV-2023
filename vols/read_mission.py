#Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from time import sleep
from Drone import Drone
import dronekit

#Instanciation d'un objet "Drone" pour contrôler le drone 
drone = Drone()
cmds = drone.vehicle.commands

try:
    while True:
        cmds.download()
        print(cmds[0])
        print("Condition validée" if (cmds[0].command == 183 and cmds[0].param1 == 10 and cmds[0].param2 == 1750) else "Condition non validée")
        sleep(2)

# On arrête l'ascencion du drone avec un Ctrl+C
except KeyboardInterrupt:
    print("Arrêt du programme")