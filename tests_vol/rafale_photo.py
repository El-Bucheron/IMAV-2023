"""
Code prenant des photos en continue avec la PiCamera

Paramètres attendus :
    (optionnel) Nom du dossier qui stockera les photos enregistrées
    
Spécificités:
    L'intervalle entre les photos est de 0.5 secondes
    La nomenclatrue des photos est la suivante : "heure:minute:seconde latitude,longitude,altitude.jpg 
    Le code s'arrête avec Ctrl+C
    Quand le drone est à moins d'un mètre du sol, aucune photo n'est prise
    Les photos prises durant un même appel de programme seront stockées dans un dossier qui sera crée en début de code
    Les dossiers de photos sont eux-mêmes contenus dans un dossier "photos" qui sera créé, s'il n'existe pas au moment de l'appel du code
"""

#Code permettant d'importer la classe "Drone"
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
    
    
# Instanciation de objet "Drone" 
drone = Drone()


print("Début de programme")
try:
    # Boucle infinue pour capture les photos en continu
    while True:
        # Si le drone est à plus d'un mètre du sol la photo est prise
        if(drone.vehicle.rangefinder.distance > 1):
            # Prise de la photo
            photo = drone.camera.prise_photo()
            # Création du chemin de la photo
            chemin_photo = (path + nom_dossier +                              # Chemin du dossier
                datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +           # Heure de prise de la photo  
                str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
                str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
                str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg")    # Encodage de l'altitude
            # Sauvegarde de la photo
            cv2.imwrite(chemin_photo, photo)
            # Temporisation
            sleep(0.2)
            print("Photo prise")

# Arret du programme avec le Ctrl+C
except KeyboardInterrupt:
    print("Fin de programme")