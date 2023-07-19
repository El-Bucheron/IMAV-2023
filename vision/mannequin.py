# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
import cv2
from math import cos, sin, pi
from time import sleep
from datetime import datetime
from commande_drone import Drone
from utilities import creation_dossier_photo, enregistrement_photo_date_position

# Connexion au drone
drone = Drone()

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)

print("Début de programme")
# Structure "try" pour pouvoir arrêter le programme avec un Ctrl+C
try:
    while True:
        # Prise de photo de la zone 
        nb_mannequins, image, result = drone.camera.detection_position(15)
        print("Mannequins détectés : " + nb_mannequins)
        # Enregistrement des photos
        enregistrement_photo_date_position(drone, image, chemin_dossier)
        enregistrement_photo_date_position(drone, result, chemin_dossier, "filtre")
        # Temporisation 
        sleep(1)
except KeyboardInterrupt:
    print("Fin de programme")
