
# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

from Drone import *

# Création de l'objet drone
drone = Drone()

# Listerner déclanchant la manoeuvre d'atterissage
@drone.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    print(message.servo10_raw)
    if(message.servo10_raw == 1350):
      drone.set_mode("GUIDED")

# Boucle d'attente de la commande "SERVO_OUTPUT_RAW" pour l'atterissage sur l'aruco
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Interruption de programme")
