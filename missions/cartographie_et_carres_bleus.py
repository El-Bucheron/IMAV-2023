"""
Description:
Code prenant des photos en vol pour la cartographie avec identification des carrés bleus

Fonctionnement:
Le code prend des photos en continu avec une fréquence de deux photos par secondes
La prise de photo est arrêtée par un Ctrl+C
Chaque photo est ensuite stockée dans le dossier "images_cartographie" situé dans le même dossier que le programme
On essaie également de chercher des carrés bleus sur l'image
Si un carré bleu est bien détecté, on sauvegarde une copie de l'image dans le dossier "carres_bleus"
"""

# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
import cv2
from time import sleep
from datetime import datetime
from commande_drone import Drone

# Import de l'objet drone
drone = Drone()


print("Début de programme")
try:
    while True:

        # Si le drone est à moins de 14 mètres d'altitude on ne prend pas de photo
        if drone.vehicle.rangefinder.distance <= 14:
            continue

        # Détection d'un aruco
        detection, image, _ = drone.camera.detection_carre_bleu()
        print("Carre bleu détecté" if detection == True else "Carre bleu non détecté")
        # Création du chemin des photos
        nom_photo = (datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +  # Heure de prise de la photo  
            str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
            str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
            str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg")    # Encodage de l'altitude
        # Sauvegarde de la photo
        cv2.imwrite("images_cartographie/"+nom_photo, image)        
        if detection == True:
           cv2.imwrite("carres_bleus/"+nom_photo, image)              
        # Temporisation prenant en comtpe le temps de prise de la photo (environ 0.33 sec) pour avoir deux photos par secondes
        sleep(0.15)

# Interruption de la boucle par ctrl+C     
except KeyboardInterrupt:
    print("Fin de programme")

