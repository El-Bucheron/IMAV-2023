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
from utilities import *
from dronekit import LocationGlobalRelative
from math import pi

altitude = 25
tracker = cv2.TrackerCSRT_create()
bbox = (0,0,0,0)
taille_carré = 100


#Instanciation de l'objet drone
drone = Drone()
drone.attente_stabilize_auto()
drone.arm_and_takeoff(altitude)
drone.goto(LocationGlobalRelative(48.7065019, 7.7343884, altitude), 0.5)


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
            # Calcul de la vitesse de la voiture
            car_center_x = int(bbox[0]+bbox[2]/2)
            car_center_y = int(bbox[1]+bbox[3]/2)
            vitesseVoitureEst, vitesseVoitureNord, carYaw = drone.get_car_speed(car_center_x, car_center_y)
            # Rotation du drone pour être dans le même sens que la voiture
            drone.set_yaw(degrees(carYaw+pi/2))
        
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    drone.set_mode("LAND")
    print("Fin de programme")