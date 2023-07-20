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
from utilities import creation_dossier_photo, enregistrement_photo_date_position, tracage_nord_est
from time import sleep
from datetime import datetime
from commande_drone import Drone


# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)
    
    
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
            # tracage_nord_est(drone, photo)
            # Enregistrement de la photo
            enregistrement_photo_date_position(drone, photo, chemin_dossier)
            # Temporisation
            sleep(0.2)
            print("Photo prise")

# Arret du programme avec le Ctrl+C
except KeyboardInterrupt:
    print("Fin de programme")
