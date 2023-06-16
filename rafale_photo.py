# import the necessary package
from time import sleep
from datetime import datetime
from commande_drone import Drone
from detection_target import Detection
import os
import sys

path = "/home/housso97/Desktop/2023/random IMAV 2023/Test vol drone 14 juin/images_prises/"

try:
    nom_dossier = sys.argv[1]
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
os.mkdir(path + nom_dossier)
    
drone = Drone()
camera = Detection()

print("Début de programme")
try:
    while True:
        camera.prise_photo(path + nom_dossier + "/"+                                     # Chemin du dossier
                           str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
                           str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
                           str('%.2f'%(drone.vehicle.rangefinder.distance)) + " " +      # Encodage de l'altitude
                           datetime.now().strftime("%H:%M:%S") + ".jpg")                 # Heure de prise de la photo
        sleep(1)
        print("Photo prise")
except KeyboardInterrupt:
    print("Fin de programme")
