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
from time import sleep
from datetime import datetime

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


drone = Drone()

print("Début de programme")
try:
    while True:
        # Détection d'un aruco
        X, _, image, image_filtree = drone.camera.detection_carre_blanc(drone.vehicle.rangefinder.distance, True)
        print(("Carré blanc détecté" if X != None else "Carré blanc non détecté") + " altitude: " + str('%.2f'%(drone.vehicle.rangefinder.distance)))
        # Création du chemin des photos
        chemin_photo = (path + nom_dossier +                              # Chemin du dossier
            datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +           # Heure de prise de la photo  
            str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
            str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
            str('%.2f'%(drone.vehicle.rangefinder.distance)) + " " +      # Encodage de l'altitude
            ("yes" if X != None else "no") + ".jpg")                      # Indique si l'aruco a été detecté ou non
        chemin_photo_filtre = (path + nom_dossier +                       # Chemin du dossier
            datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +           # Heure de prise de la photo  
            str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
            str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
            str('%.2f'%(drone.vehicle.rangefinder.distance)) + " " +      # Encodage de l'altitude
            ("yes" if X != None else "no") + " filtre.jpg")               # Indique si l'aruco a été detecté ou non
        # Sauvegarde de la photo
        cv2.imwrite(chemin_photo, image)
        cv2.imwrite(chemin_photo_filtre, image_filtree)
        # Temporisation 
        sleep(1)
except KeyboardInterrupt:
    print("Fin de programme")
