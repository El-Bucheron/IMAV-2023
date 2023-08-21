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

#Instanciation de l'objet drone
drone = Drone()

# Listerner déclanchant la manoeuvre d'atterissage
@drone.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):

    # Passage et attente en mode "GUIDED"    
    self.set_mode("GUIDED")
    while self.get_mode() != "GUIDED":
        pass

    print("Début de la manoeuvre d'atterissage")
    try:
        drone.atterrissage_aruco_fonctionnel(chemin_dossier)
    except Exception as e:
        print(e)
    finally:
        sys.exit(0) 
      

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
drone.attente_stabilize_auto()  


# Boucle d'attente de la commande "SERVO_OUTPUT_RAW"
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Fin de programme")