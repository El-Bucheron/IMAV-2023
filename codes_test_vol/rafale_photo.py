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
    Les précedents dossiers sont eux-mêmes contenus dans un dossier "photos" qui sera créé s'il n'existe pas au moment de l'appel du code
"""
#Code permettant d'importer les classes "Drone" et "Detection"
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
from detection_target import Detection


# Chemins absolu du dossier contenant les dossiers de photos
path = os.path.dirname(os.getcwd()) + "/photos/"
# Si ce dossier n'existe pas, on le créé
try:
    os.mkdir(path)
# Si le dossier existe déjà, on ne fait rien    
except FileExistsError:
    pass


#On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
try:
    nom_dossier = sys.argv[1] + "/"
#Si l'utilisateur n'a pas fourni de nom de dossier, on utilse la date et l'heure d'appel du code pour le nommer    
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S") + "/"

#Création du dossier
try:
    os.mkdir(path + nom_dossier)
#Si le fichier existe déjà, on ne fait rien
except FileExistsError:
    pass
    
    
# Instanciation d'un objet de type drone pour obtenir ses coordonnées GPS et d'un objet de type Detection pour prendre les photos   
drone = Drone()
camera = Detection()


print("Début de programme")
try:
    # Boucle infinue pour capture les photos en continu
    while True:
        # Si le drone est à plus d'un mètre du sol la photo est prise
        if(drone.vehicle.rangefinder.distance > 1):
            # Prise de la photo et stockage selon la nomenclatrue décrite en début de code 
            camera.prise_photo(path + nom_dossier +                                          # Chemin du dossier
                               datetime.now().strftime("%H:%M:%S") + " " +                   # Heure de prise de la photo  
                               str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
                               str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
                               str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg ")   # Encodage de l'altitude
            # Temporisation
            sleep(0.5)
            print("Photo prise")

# Arret du programme avec le Ctrl+C
except KeyboardInterrupt:
    print("Fin de programme")
