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

drone = Drone()

print("Début de programme")
# Structure "try" pour pouvoir arrêter le programme avec un Ctrl+C
try:
    while True:
        # Détection d'un aruco
        detection, image, _ = drone.camera.detection_carre_bleu()
        print("Carre bleu détecté" if detection == True else "Carre bleu non détecté")
        # Création du chemin des photos
        nom_photo = (datetime.now().strftime("%H:%M:%S") + " " +          # Heure de prise de la photo  
            str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
            str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
            str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg")    # Encodage de l'altitude
        # Sauvegarde de la photo
        cv2.imwrite("images_cartographie/"+nom_photo, image)        
        if detection == True:
           cv2.imwrite("carres_bleus/"+nom_photo, image)              
        # Temporisation 
        sleep(1)
        
except KeyboardInterrupt:
    print("Fin de programme")

