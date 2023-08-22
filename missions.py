import os
from detection_target import *
from utilities import *
from commande_drone import *

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
            drone.atterrissage_aruco_fonctionnel(chemin_atterrissage)
        except Exception as e:
            print(e)
        finally:
            sys.exit(0) 


# Choix de la mission
numero_mission = int(input("Quel mission voulez-vous lancer ?\n"+
      "1) Cartographie\n" + 
      "2) L'inspection dynamique\n" + 
      "3) Identification de l'état des randonneurs\n" +
      "4) Asservissement sur aruco\n" +
      "5) Arrêt du programme")) 
while numero_mission not in [1,2,3,4,5]:
    numero_mission = input("Numéro de mission non reconnu. Veuillez resaisir le numéro")


#Cartographie
if numero_mission == 1:

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



# Inspection dynamique
elif numero_mission == 2:
    # Boucle d'attente de la commande "SERVO_OUTPUT_RAW" pour l'atterissage sur l'aruco
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Interruption de programme")



# Identification de l'état des randonneurs
elif numero_mission == 3:

    #Choix de l'altitude de vol : 
    altitude = 15
    # Attente du mode stabilize puis du mode auto
    drone.attente_stabilize_auto()
    # Décollage
    drone.arm_and_takeoff(altitude)
    #Vol vers la zone où se trouvent les mannequins (coordonnées de la compète)
    drone.goto(LocationGlobalRelative(50.909228, 6.226700, altitude), 0.5)

    # Temporisation pour la stabilisation de la position et du drone
    sleep(5)

    # Prise de photo de la zone 
    nb_mannequins, image, result = drone.camera.detection_position(altitude)
    print(nb_mannequins)

    # Enregistrement des photos
    chemin_dossier = creation_dossier_photo("Detection mannequins")
    enregistrement_photo_date_position(drone, image, chemin_dossier)
    enregistrement_photo_date_position(drone, result, chemin_dossier, "filtre")

    # Boucle d'attente de la commande "SERVO_OUTPUT_RAW" pour l'atterissage sur l'aruco
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Interruption de programme")


# Asservissement
elif numero_mission == 4:

    altitude = 20
    # Attente du mode stabilize puis du mode auto
    drone.attente_stabilize_auto()
    # Décollage
    drone.arm_and_takeoff(altitude)
    #Vol vers la zone où se trouvent les mannequins (coordonnées de la compète)
    #drone.goto(LocationGlobalRelative(50.910031, 6.226700, 25), 0.5)
    drone.goto(LocationGlobalRelative(48.7065019, 7.7343884, altitude), 0.5)

    chemin_dossier = creation_dossier_photo("Suivi de véhicule : " + datetime.now().strftime("%d-%m %H:%M:%S"))
    while True:
        # Détection de l'aruco
        aruco_center_x, aruco_center_y, _, image = drone.camera.detection_aruco(True)
        print(("Aruco trouvé de centre X = " + str(aruco_center_x) + " ; Y = " + str(aruco_center_y)) if aruco_center_x != None else "Aruco non détecté")
        # Asservissement par rapport au centre de l'aruco
        erreurX, erreurY, vx, vy = drone.asservissement_suivi_vehicule_fonctionnel(aruco_center_x, aruco_center_y)      
        # Affichage de l'erreur et de la vitesse
        image = cv2.putText(image, "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        image = cv2.putText(image, "Vitesse : Vx = " + str(vx) + " ; Vy = " + str(vy), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)              
        # Traçage d'un cercle au centre de l'image
        cv2.circle(image, (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 4, (0, 255, 0), -1)
        # Sauvegarde de la photo
        enregistrement_photo_date_position(drone, image, chemin_dossier, "yes" if aruco_center_x != None else "no")
    


# Arrêt du programme
elif numero_mission == 5:
    sys.exit(0)