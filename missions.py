import os, sys, traceback
from Detection import *
from utilities import *
from Drone import *

# See https://stackoverflow.com/a/57008707
class Logger(object):
    def __init__(self, filename):
        self.file = open(os.path.join("logger_output",filename), 'w')
        self.stdout = sys.stdout

    def __enter__(self):
        sys.stdout = self

    def __exit__(self, exc_type, exc_value, tb):
        sys.stdout = self.stdout
        if exc_type is not None:
            self.file.write(traceback.format_exc())
        self.file.close()

    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)

    def flush(self):
        self.file.flush()
        self.stdout.flush()


# Création de l'objet drone
drone = Drone()


# Listerner déclanchant la manoeuvre d'atterissage
@drone.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):

    # Condition de déclenchement de la manoeuvre d'atterissage
    if int(message.servo10_raw) == 1350:
        
        # Passage et attente en mode "GUIDED"    
        self.set_mode("GUIDED")
        while self.get_mode() != "GUIDED":
            pass

        # Atterrissage
        print("Début de la manoeuvre d'atterissage")
        try:
            drone.atterrissage_aruco(chemin_atterrissage)
        except Exception as e:
            print(e)
        finally:
            sys.exit(0) 


# Choix de la mission
numero_mission = int(input("Quel mission voulez-vous lancer ?\n"+
    "1) Cartographie\n" + 
    "2) Identification de l'état des randonneurs\n" +
    "3) Asservissement sur aruco\n" +
    "4) Arrêt du programme\n"))
while numero_mission not in range(1,5):
    numero_mission = input("Numéro de mission non reconnu. Veuillez resaisir le numéro")


#Cartographie
if numero_mission == 1:
    with Logger("Planification dynamique : " + datetime.now().strftime("%d-%m %H:%M:%S") + ".txt"):
        print("Début de la cartographie")

        #Création des dossiers de prises de photos
        chemin_dossier = creation_dossier_photo("Planification dynamique : " + datetime.now().strftime("%d-%m %H:%M:%S"))
        chemin_carto = os.path.join(chemin_dossier, "cartographie")
        chemin_carre_bleu = os.path.join(chemin_dossier, "carre_bleu")
        chemin_atterrissage = os.path.join(chemin_dossier, "atterrisssage")
        os.mkdir(chemin_carto)
        os.mkdir(chemin_carre_bleu)
        os.mkdir(chemin_atterrissage)


        try:
            # Boucle de prise de photo
            while True:
                # Si le drone est à moins de 25 mètres d'altitude on ne prend pas de photo
                if drone.vehicle.rangefinder.distance <= 25:
                    print("drone trop bas")
                    sleep(0.5)
                    continue
                
                # Détection d'un aruco
                detection, image = drone.camera.detection_carre_bleu()
                print("Photo prise : " + ("Carre bleu détecté" if detection == True else "Carre bleu non détecté"))
                enregistrement_photo_date_position(drone, image, chemin_carto)
                if detection == True:
                    enregistrement_photo_date_position(drone, image, chemin_carre_bleu)             
                    # Temporisation prenant en compte le temps de prise de la photo (environ 0.3 sec) pour avoir deux photos par secondes
                    sleep(0.2)
                    
        # Interruption de la boucle par ctrl+C     
        except KeyboardInterrupt:
            print("Interruption de la cartographie")


# Identification de l'état des randonneurs
elif numero_mission == 2:
    with Logger("Detection mannequins : " + datetime.now().strftime("%d-%m %H:%M:%S") + ".txt"):

        #Choix de l'altitude de vol : 
        altitude = 15
        # Attente du mode stabilize puis du mode auto
        drone.attente_stabilize_auto()
        # Récupération de la position initiale du drone
        position_initiale = LocationGlobalRelative(drone.vehicle.location.global_frame.lat, drone.vehicle.location.global_frame.lon, altitude)
        # Décollage
        drone.arm_and_takeoff(altitude)
        #Vol vers la zone où se trouvent les mannequins (coordonnées de la compète)
        #drone.goto(LocationGlobalRelative(50.909228, 6.226700, altitude), 0.5)
        drone.goto(LocationGlobalRelative(48.70652, 7.73549, altitude), 0.5)
        
        # Temporisation pour la stabilisation de la position et du drone
        sleep(5)
        
        # Prise de photo de la zone 
        nb_mannequins, image, result = drone.camera.detection_position(altitude)
        print(nb_mannequins)

        # Enregistrement des photos
        chemin_dossier = creation_dossier_photo("Detection mannequins : " + datetime.now().strftime("%d-%m %H:%M:%S"))
        enregistrement_photo_date_position(drone, image, chemin_dossier)
        enregistrement_photo_date_position(drone, result, chemin_dossier, "filtre")
        
        # Retour du drone à sa position initiale
        drone.goto(position_initiale,0.25)
        
        # Boucle d'attente de la commande "SERVO_OUTPUT_RAW" pour l'atterissage sur l'aruco
        print("Début de la manoeuvre d'atterissage")
        try:
            drone.atterrissage_aruco(chemin_dossier)
        except Exception as e:
            print(e)
        finally:
            sys.exit(0) 
            

# Asservissement
elif numero_mission == 3:
    with Logger("Suivi de véhicule : " + datetime.now().strftime("%d-%m %H:%M:%S") + ".txt"):

        altitude = 25
        # Attente du mode stabilize puis du mode auto
        drone.attente_stabilize_auto()
        # Décollage
        drone.arm_and_takeoff(altitude)
        #Vol vers la zone où se trouvent les mannequins (coordonnées de la compète)
        drone.goto(LocationGlobalRelative(50.910031, 6.226700, 25), 0.5)
        #drone.goto(LocationGlobalRelative(48.7066603, 7.7341269, altitude), 0.5)
        # Création du dossier recevant les photos
        chemin_dossier = creation_dossier_photo("Suivi de véhicule : " + datetime.now().strftime("%d-%m %H:%M:%S"))
        # Initialisation du suivi de véhicule
        drone.suivi_vehicule(chemin_dossier)


# Arrêt du programme
elif numero_mission == 4:
    drone.vehicle.play_tune("AAAA")
    sys.exit(0)
