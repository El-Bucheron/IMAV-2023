"""
Description:
Code peremttant de tester l'asservissement pour le suivi de véhicule

Spécificités:
On attend tout d'abord que le drone soit mis en "GUIDED". 
Pendant ce temps, on affiche que le programme est en attente du mode "GUIDED", l'altitude du drone et si un aruco est détecté
Le code ne prend pas en compte le cas où l'aruco n'est pas détecté. Il faut donc commencer dans une configuration où l'aruco est détecté.
On appelle ensuite la fonction initiant l'asservissement du drone par rapport à l'aruco.
L'asservissement est arrêté par un Ctrl+C
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
from commande_drone import Drone
from datetime import datetime
from time import sleep
from utilities import creation_dossier_photo, enregistrement_photo_date_position

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)

#Instanciation de l'objet drone
drone = Drone()
drone.attente_stabilize_auto()
drone.takeoff(15)
sleep(2)

try:
    print("Début de l'assservissement")
    while True:
        # Détection de l'aruco
        aruco_center_x, aruco_center_y, _, image = drone.camera.detection_aruco(True)
        print(("Aruco trouvé de centre X = " + str(aruco_center_x) + " ; Y = " + str(aruco_center_y)) if aruco_center_x != None else "Aruco non détecté")
        # Traçage d'un cercle au centre de l'image
        cv2.circle(image, (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 4, (0, 255, 0), -1)
        # Sauvegarde de la photo
        enregistrement_photo_date_position(drone, image, chemin_dossier, "yes" if aruco_center_x != None else "no")                
        # Asservissement du drone
        drone.asservissement_suivi_vehicule(aruco_center_x, aruco_center_y)
        
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    drone.set_mode("LAND")
    print("Fin de programme")
