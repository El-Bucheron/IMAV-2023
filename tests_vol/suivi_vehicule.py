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
from commande_drone import Drone
from datetime import datetime
from time import sleep
import cv2

# Chemins absolu du dossier contenant les dossiers de photos
path = package_path + "/photos/"

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1] + "/"  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S") + "/"

# On crée le dossier de global photo s'il n'existe pas déjà
try:
    os.mkdir(path)
except FileExistsError:
    pass
# On crée le dossier de photo lié à cet appel de code s'il n'existe pas déjà
try:
    os.mkdir(path + nom_dossier)
except FileExistsError:
    pass

#Instanciation de l'objet drone
drone = Drone()

# Attente du mode "STABIIZE" puis du mode "STABILIZE"
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
while drone.get_mode() != "AUTO":    
    print("En attente du mode AUTO")
drone.set_mode("GUIDED")
while drone.get_mode() != "GUIDED":
    pass
drone.takeoff(15)
sleep(2)

try:
    print("Début de l'assservissement")
    while True:
        aruco_center_x, aruco_center_y, _, image = drone.camera.detection_aruco(True)
        print(("Aruco trouvé de centre X = " + str(aruco_center_x) + " ; Y = " + str(aruco_center_y)) if aruco_center_x != None else "Aruco non détecté")
        
        # Création du chemin des photos
        chemin_photo = (path + nom_dossier +                                          # Chemin du dossier
                        datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +           # Heure de prise de la photo  
                        str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
                        str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
                        str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg")    # Encodage de l'altitude
        # Traçage d'un cercle au centre de l'image
        cv2.circle(image, (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 4, (0, 255, 0), -1)
        # Sauvegarde de la photo
        cv2.imwrite(chemin_photo, image)        
        
        # Asservissement du drone
        drone.asservissement_suivi_vehicule(aruco_center_x, aruco_center_y)
        
# Arrêt de l'asservissement avec un Ctrl+C
except KeyboardInterrupt:
    drone.set_mode("LAND")
    print("Fin de programme")
