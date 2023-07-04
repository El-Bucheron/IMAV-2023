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

drone = Drone()

print("Début de programme")

try:
    while True:
        X,Y,aruco_id = drone.camera.detection_aruco()
        if X != None:
            word_detected = "yes"
        else:
            word_detected = "no"
        drone.camera.prise_photo(path + nom_dossier +                                          # Chemin du dossier
                                 datetime.now().strftime("%H:%M:%S") + " " +                   # Heure de prise de la photo  
                                 str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
                                 str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
                                 str('%.2f'%(drone.vehicle.rangefinder.distance)) + "," +
                                 word_detected + ".jpg ")   # Encodage de l'altitude
        print(("Aruco détecté" if X != None else "Aruco non détecté")+ " with id " + str(aruco_id) + " altitude: " + str('%.2f'%(drone.vehicle.rangefinder.distance)))
        sleep(1)

except KeyboardInterrupt:
    print("Fin de programme")
