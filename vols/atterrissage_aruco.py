
# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from utilities import creation_dossier_photo
from commande_drone import Drone
from datetime import datetime
from time import sleep

#Instanciation de l'objet drone
drone = Drone()

global boolean
boolean = True

# Listerner déclanchant la manoeuvre d'atterissage
@drone.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):
    # Condition de déclenchement de la manoeuvre d'atterissage
    if int(message.servo10_raw) == 1350:
        global boolean 
        boolean = False
    print(boolean)

# On recupère le nom de dossier fourni par l'utilisateur s'il en a fourni un
# Sinon on utilse la date et l'heure d'appel du code pour le nommer  
try:
    nom_dossier = sys.argv[1]  
except IndexError:
    nom_dossier = datetime.now().strftime("%d-%m %H:%M:%S")
# Création du dossier de photos
chemin_dossier = creation_dossier_photo(nom_dossier)


# Attente du mode auto puis du mode stabilize
print("Début de programme")

# Boucle d'attente de la commande "SERVO_OUTPUT_RAW"
try:
    while boolean:
        print("En attente de la consigne d'atterrissage")
        sleep(1)
    drone.set_mode("GUIDED")
    while drone.get_mode() != "GUIDED":
        pass
    print("Atterrissage")
    try: 
        drone.atterrissage_aruco_fonctionnel(chemin_dossier)
    except Exception as e:
        print(e)
    finally:
        sys.exit(0)
except KeyboardInterrupt:
    print("Fin de programme")
