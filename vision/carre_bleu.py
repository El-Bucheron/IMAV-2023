# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from time import sleep
from datetime import datetime
from Drone import Drone
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
        detection, image, image_filtree = drone.camera.detection_carre_bleu()
        print(("Carre bleu détecté" if detection == True else "Carre bleu non détecté") + "\naltitude: " + str('%.2f'%(drone.vehicle.rangefinder.distance)))
        # Enregistrement des photos
        enregistrement_photo_date_position(drone, image, chemin_dossier, "yes" if detection == True else "no")
        enregistrement_photo_date_position(drone, image_filtree, chemin_dossier, ("yes" if detection == True else "no") + " filtre")
        # Temporisation 
        sleep(1)
except KeyboardInterrupt:
    print("Fin de programme")
