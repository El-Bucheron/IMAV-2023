# Code permettant d'importer la classe "Drone" et de réaliser la mission de détection d'objets

#Librairies nécessaires
import os
import sys
import cv2
import numpy as np
import imutils
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from commande_drone import Drone
from time import sleep
from dronekit import LocationGlobalRelative
from utilities import enregistrement_photo_date_position, creation_dossier_photo
from datetime import datetime

#Instanciation de l'objet drone
drone = Drone()
drone.attente_stabilize_auto()

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)
    
#Choix de l'altitude de vol : 
altitude = 15
# Décollage
drone.arm_and_takeoff(altitude)
#Choix de la zone de vol : le jour de la compétition, les coordonnées GPS du lieu des mannequins : Lat: 50.909228° Lon: 6.226700°
point = LocationGlobalRelative(48.70652, 7.73407, altitude)
#Vol vers la zone où se trouve les mannequins
drone.goto(point, 1)

# Prise de photo de la zone 
nb_mannequins, image, result = drone.camera.detection_position(altitude)
print(nb_mannequins)

# Temporisation
sleep(0.5)
print("Photo prise")

# Enregistrement des photos
enregistrement_photo_date_position(drone, image, chemin_dossier)
enregistrement_photo_date_position(drone, result, chemin_dossier, "filtre")
    
#Retour à la base
drone.set_mode("RTL")
