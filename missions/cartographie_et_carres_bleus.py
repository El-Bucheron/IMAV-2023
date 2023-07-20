"""
Description:
Code prenant des photos en vol pour la cartographie avec identification des carrés bleus

Fonctionnement:
Le code prend des photos en continu avec une fréquence de deux photos par secondes
La prise de photo est arrêtée par un Ctrl+C
Chaque photo est ensuite stockée dans le dossier "images_cartographie" situé dans le même dossier que le programme
On essaie également de chercher des carrés bleus sur l'image
Si un carré bleu est bien détecté, on sauvegarde une copie de l'image dans le dossier "carres_bleus"
"""

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
from commande_drone import Drone
from utilities import enregistrement_photo_date_position, creation_dossier_photo

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)

chemin_carto = os.path.join(chemin_dossier, "cartographie")
chemin_carre_bleu = os.path.join(chemin_dossier, "carre_bleu")
os.mkdir(chemin_carto)
os.mkdir(chemin_carre_bleu)


# Import de l'objet drone
drone = Drone()


print("Début de programme")
try:
    while True:

        # Si le drone est à moins de 14 mètres d'altitude on ne prend pas de photo
        if drone.vehicle.rangefinder.distance <= 0.5:
            print("drone trop bas")
            sleep(0.5)
            continue

        # Détection d'un aruco
        detection, image = drone.camera.detection_carre_bleu()
        print("Photo prise : " + ("Carre bleu détecté" if detection == True else "Carre bleu non détecté"))
        enregistrement_photo_date_position(drone, image, chemin_carto)
        if detection == True:
            enregistrement_photo_date_position(drone, image, chemin_carre_bleu)             
        # Temporisation prenant en comtpe le temps de prise de la photo (environ 0.3 sec) pour avoir deux photos par secondes
        sleep(0.2)

# Interruption de la boucle par ctrl+C     
except KeyboardInterrupt:
    print("Fin de programme")

