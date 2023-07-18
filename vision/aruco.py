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
from utilities import creation_dossier_photo, enregistrement_photo_date_position

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)


drone = Drone()

print("Début de programme")
# Structure "try" pour pouvoir arrêter le programme avec un Ctrl+C
try:
    while True:
        # Détection d'un aruco
        centre_aruco_X, centre_aruco_Y, aruco_id, image = drone.camera.detection_aruco(True)
        print(("Aruco détecté" if centre_aruco_X != None else "Aruco non détecté")+ " with id " + str(aruco_id) + " altitude: " + str('%.2f'%(drone.vehicle.rangefinder.distance)))
        erreurX, erreurY, vx, vy = drone.asservissement_atterrissage_2022(centre_aruco_X, centre_aruco_Y)
        message_image = "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY) + "\nVitesse : Vx = " + str(vx) + " ; Vy = " + str(vy)
        cv2.putText(image, message_image, (0, 0), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        # Traçage d'un cercle au centre de l'image
        cv2.circle(image, (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 4, (0, 255, 0), -1)
        # Sauvegarde de la photo
        enregistrement_photo_date_position(drone, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")
        # Temporisation 
        sleep(1)
except KeyboardInterrupt:
    print("Fin de programme")
