# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
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


# On crée le dossier de global photo et/ou le dossier de photo e l'appel de code s'ils n'existent pas déjà
try:
    os.mkdir(path)
except FileExistsError:
    pass
try:
    os.mkdir(path + nom_dossier)
except FileExistsError:
    pass


#Instanciation de l'objet drone
drone = Drone()

# Attente du mode "STABILIZE" puis du mode "GUIDED"
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
while drone.get_mode() != "GUIDED":    
    print("En attente du mode GUIDED")
    
# Décollage du drone
print("Décollage")
drone.takeoff(20)
sleep(10)


print("Début de la manoeuvre d'atterissage")
try:
    drone.atterrissage_aruco_matthieu(path + nom_dossier)  
except KeyboardInterrupt:
    drone.set_mode("LAND")
    print("Fin de programme")