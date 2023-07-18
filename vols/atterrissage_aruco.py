# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from utilities import creation_dossier_photo, enregistrement_photo_date_position
from commande_drone import Drone
from time import sleep
from datetime import datetime

#Instanciation de l'objet drone
drone = Drone()
    
print("Début de programme")
try:
    while drone.vehicle.rangefinder.distance > 6 and drone.get_mode != "LOITER":
        print("En attente du mode LOITER pour atterrir")
        sleep(1)

    # On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
    # Sinon on utilse la date et l'heure d'appel du code pour le nommer  
    try:
        nom_dossier = sys.argv[1]  
    except IndexError:
        nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
    # Création du dossier de photos
    chemin_dossier = creation_dossier_photo(nom_dossier)

    print("Début de la manoeuvre d'atterissage")
    drone.atterrissage_aruco_fonctionnel(chemin_dossier)  

except KeyboardInterrupt:
    print("Fin de programme")
