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
from utilities import creation_dossier_photo, enregistrement_photo_date_position, tracage_nord_est

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
        # Détection d'un aruco
        centre_aruco_X, centre_aruco_Y, aruco_id, image = drone.camera.detection_aruco(True)
        print("Aruco détecté" if centre_aruco_X != None else "Aruco non détecté")
        # Calcul de l'asservissement 
        erreurX, erreurY, erreurNord, erreurEst, VEst, VNord = drone.asservissement_atterrissage_metres(centre_aruco_X, centre_aruco_Y)
        print("Erreur en pixels : EX = " + str(erreurX) + " ; EY = " + str(erreurY))
        print("Erreur NORD-EST: EN = " + str(erreurNord) + " ; EE = " + str(erreurEst))
        print("Vitesse NORD-EST : VN = " + str(VNord) + " ; VE = " + str(VEst))
        # Affichage de l'erreur et de la vitesse
        cv2.putText(image, "Erreur XY: EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        cv2.putText(image, "Erreur NORD-EST: EN = " + str(erreurNord) + " ; EE = " + str(erreurEst), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        cv2.putText(image, "Vitesse NORD-EST : VN = " + str(VNord) + " ; VE = " + str(VEst), (0, 75), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        tracage_nord_est(drone, image)
        # Sauvegarde de la photo
        enregistrement_photo_date_position(drone, image, chemin_dossier)
        # Temporisation 
        sleep(1)
except KeyboardInterrupt:
    print("Fin de programme")
