# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
import cv2
from Drone import Drone
from datetime import datetime
from utilities import *
from dronekit import LocationGlobalRelative


altitude = 25
tracker = cv2.TrackerCSRT_create()
bbox = (0,0,0,0)
taille_carré = 100


#Instanciation de l'objet drone
drone = Drone()
drone.attente_stabilize_auto()
drone.arm_and_takeoff(altitude)
drone.goto(LocationGlobalRelative(48.7065019, 7.7343884, altitude), 0.5)

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)


try:
    print("Début de la détection")
    # Bouche infinie servant à chercher l'aruco pour initialiser le tracker
    while True:
        # Détection de l'aruco
        aruco_center_x, aruco_center_y, id, image = drone.camera.detection_aruco(True)
        # Si un aruco a été détecté et que son ID est 700, on intialise le tracker grace au centre trouvé et on sort de la bouche
        if aruco_center_x != None and id == 700:
            # Détermination de la zone d'intérêt : un carré de même centre que celui de l'aruco et de côté "taille_carré"
            bbox = (aruco_center_x-int(taille_carré/2),aruco_center_y-int(taille_carré/2), taille_carré, taille_carré)
            # Initialisation du tracker
            tracker.init(image, bbox)
            # Sortie de boucle
            break

    print("Aruco détecté")

    # Boucle infinie asservisement le drone par rapport au centre de l'objet tracké
    while True:

        # Prise de photo
        image = drone.camera.prise_photo()
        # Tracking 
        ok, bbox = tracker.update(image)
        print("Voiture détectée" if ok else "Voiture non détectée")
        if ok:
            GPS = get_GPS_through_picture(drone, int(bbox[0]+bbox[2]/2), int(bbox[1]+bbox[3]/2))
            GPS.alt = altitude
            print("Coordonnées obtenues : lat : " + str(GPS.lat) + " ; lon : " + str(GPS.lon))
            print("Début du déplacement")
            drone.goto(GPS, 0.5)
            print("Fin du déplacement")
            cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])), (0, 0, 255), 2, 2) 
        enregistrement_photo_date_position(drone, image, chemin_dossier, "yes" if ok else "no")


        
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    drone.set_mode("LAND")
    print("Fin de programme")
