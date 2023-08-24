#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""

import cv2
from time import sleep
from math import atan2, cos, sin, sqrt, tan, radians
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from utilities import *
from detection_target import Detection



class Drone:

#-------------------------------------------------------------------------------------------------------------------------------------------
# Initialisation de la classe 
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Constructeur de la classe se connectant au drone
    def __init__(self):     
        
        # Coefficients de l'asservissement PID de l'atterrissage
        self.kp_atterrissage = 0 # Coefficient mis à 0 car initialisé plus tard
        self.kd_atterrissage = 0.0002  # 0.00001 working "fine" for both
        self.ki_atterrissage = 0.000001  # 0.0000001
        self.coefficient_kp_atterrissage = 0.5
        self.offset_camera_atterrissage = 0
        self.coef_vx_atterrissage = 1
        self.coef_vy_atterrissage = 1

        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX_atterrissage = 0
        self.erreurIntegraleY_atterrissage = 0
        self.erreurAnterieureX_atterrissage = 0
        self.erreurAnterieureY_atterrissage = 0

        # Coefficients de l'asservissement PID du suivi de véhicule
        self.kp_suivi_vehicule = 0.0125/2.0
        self.kd_suivi_vehicule = 0.000125*5  # 0.00001 working "fine" for both
        self.ki_suivi_vehicule = 0.000002 #0.0000001
        self.offset_camera_suivi_vehicule = 0
        self.coef_vx_suivi_vehicule = 1
        self.coef_vy_suivi_vehicule = 1
        self.compteur_non_detection = 0
        self.stored_vx = 0
        self.stored_vy = 0
        
        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX_suivi_vehicule = 0
        self.erreurIntegraleY_suivi_vehicule = 0
        self.erreurAnterieureX_suivi_vehicule = 0
        self.erreurAnterieureY_suivi_vehicule = 0

        self.previousGPSlocation = 0

        
        # Connexion au drone et initialisation de la caméra
        print("Connexion au drone et initialisation de la caméra")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=2)
        self.camera = Detection()
        print("Connexion et initialisation terminées")

        # Envoi de consigne de servo pour la condition d'atterrissage
        msg = self.vehicle.message_factory.command_long_encode(0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            10,  # servo number
            1900,  # servo position between 1000 ferme and 2000
            0, 0, 0, 0, 0)  # param 3 ~ 7 not used
        self.vehicle.send_mavlink(msg)


    #set_mode - set the mode of the vehicle as long as we are in control
    def set_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.flush()
            
    #get_mode - get current mode of vehicle 
    def get_mode(self):
        return self.vehicle.mode.name




#-------------------------------------------------------------------------------------------------------------------------------------------
# Fonctions de pilotage du drone
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction demandant au drone de passer en mode "STABILIZE" puis en mode "AUTO" et enfin passe le drone en mode "GUIDED"
    # Cette fonction est utilisée comme sécurité avant un code de pilotage de drone 
    # Elle permet de passer à la suite du code en changement adéquatement de mode avec la télécommande
    # Après avoir obtenu la bonne séquence de modes, elle passage également le drone en mode "GUIDED" pour contrôle le drone avec dronekit
    def attente_stabilize_auto(self):

        # Attente du mode "STABIIZE"
        while self.get_mode() != "STABILIZE":
            print("En attente du mode STABILIZE")
            sleep(1)

        # Attente du mode "AUTO"
        while self.get_mode() != "AUTO":    
            print("En attente du mode AUTO")
            sleep(1)

        # Passage en mode "GUIDED"    
        self.set_mode("GUIDED")
        # Attente du passage effectif en mode "GUIDED"
        while self.get_mode() != "GUIDED":
            pass



    # Fonction faisant décoler le drone à l'altitude passée en argument
    def takeoff(self, altitude):
        # Ordre de décollage du drone
        self.vehicle.simple_takeoff(altitude)
        # On attend que le drone soit à 95% de l'altitude souhaitée pour sortir de la fonction,
        # car la fonction "simple_takeoff" ne bloque pas le déroulement du programme 
        while self.vehicle.rangefinder.distance < 0.95*altitude:
            pass


    # Fonction déplaçant le drone à la position GPS fournie en argument
    def goto_2(self, targetLocation, distanceAccuracy):
        # Ordre de déplacement du drone
        self.vehicle.simple_goto(targetLocation)
        # On attend que le drone soit arrivé assez près du point 
        # car la fonction "simple_goto" ne bloque pas le déroulement du programme 
        while get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation) > distanceAccuracy:
            pass


    # Décollage du drone jusqu'à la distance fournie en argument
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            sleep(1)

        print("Taking off!")
        print(aTargetAltitude)
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            sleep(1)
                  
   
    # Définition de la consigne de vitesse selon le repère x,y,z du drone et pendant 0.1 seconde 
    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
          0,  # time_boot_ms (not used)
          0, 0,  # target system, target component
          mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
          0x0DC7,  # type_mask (ignore pos | ignore acc)
          0, 0, 0,  # x, y, z positions (not used)
          velocity_x, velocity_y, velocity_z,
          # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
          0, 0, 0,  # x, y, z acceleration (not used)
          0, 0)  # yaw, yaw_rate (not used)
        # Envoie de la consigne de vitesse au drone 
        self.vehicle.send_mavlink(msg)
        # Temporisation de 0.1 seconde
        sleep(0.1)

                       
    def goto(self, targetLocation, distanceAccuracy):
        # Simple goto DroneKit function
        self.vehicle.simple_goto(targetLocation)

        # Stop action if we are no longer in GUIDED mode
        while self.vehicle.mode.name == "GUIDED": 
            currentLocation = self.vehicle.location.global_relative_frame
            remainingDistance = get_distance_metres(currentLocation, targetLocation)
            print ("[mission] Distance to the GPS target: ", remainingDistance)
            # print("Distance to the GPS target: %.2fm" % d)

            # If the distance to the target verifies the distance accuracy
            if remainingDistance <= distanceAccuracy:
                print("[mission] Reached GPS target!")
                break  # Then break the waiting loop
            sleep(1)

#-------------------------------------------------------------------------------------------------------------------------------------------
# Asservissements
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_suivi_vehicule_fonctionnel(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            self.compteur_non_detection += 1
            if self.compteur_non_detection >= 5:
                return None, None, None, None
            self.set_velocity(self.stored_vy, self.stored_vx, 0)
            return None, None, self.stored_vx, self.stored_vy

        
        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = self.camera.x_imageCenter - aruco_center_x
        erreurY = self.camera.y_imageCenter - aruco_center_y + self.offset_camera_suivi_vehicule
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurX = dist_center * cos(alpha)
        erreurY = dist_center * sin(alpha)
        # Si l'erreur selon x et y est inférieure à 10 pixels, on la considère comme nulle
        if abs(erreurX) <= 10:  
            erreurX = 0
        if abs(erreurY) <= 10:
            erreurY = 0

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeX = (erreurX - self.erreurAnterieureX_suivi_vehicule)
        erreurDeriveeY = (erreurY - self.erreurAnterieureY_suivi_vehicule)
        # Erreur intégrale
        self.erreurIntegraleX_suivi_vehicule += erreurX
        self.erreurIntegraleY_suivi_vehicule += erreurY
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureX_suivi_vehicule = erreurX
        self.erreurAnterieureY_suivi_vehicule = erreurY

        # Calcul de la vitesse corrigée 
        vx = self.kp_suivi_vehicule * erreurX + self.kd_suivi_vehicule * erreurDeriveeX + self.ki_suivi_vehicule * self.erreurIntegraleX_suivi_vehicule
        vy = self.kp_suivi_vehicule * erreurY + self.kd_suivi_vehicule * erreurDeriveeY + self.ki_suivi_vehicule * self.erreurIntegraleY_suivi_vehicule        
        # Correction en fonction de l'angle 
        #vx *= abs(cos(self.vehicle.attitude.roll) ** self.coef_vx_suivi_vehicule)
        #vy *= abs(cos(self.vehicle.attitude.pitch) ** self.coef_vy_suivi_vehicule)
        vitesseDroneX, vitesseDroneY = self.calcul_vitesse_drone()
        vitesseEstX = - vx - vitesseDroneX
        vitesseNordY = vy + vitesseDroneY
        
        #Envoie de la consigne de vitesse au drone
        self.set_velocity(vitesseNordY, vitesseEstX, 0) # Pour le sense de la camera, X pointe vers l'est et Y vers le nord
        self.compteur_non_detection = 0
        self.stored_vx = vitesseEstX
        self.stored_vy = vitesseNordY
        return erreurX, erreurY, vitesseEstX, vitesseNordY
    

    def calcul_vitesse_drone(self):
        vitesseEstX = 0
        vitesseNordY = 0
        if self.previousGPSlocation != 0:
            groundSpeed = self.vehicle.groundspeed
            angle = atan2(self.vehicle.location._lat - self.previousGPSlocation._lat, self.vehicle.location._lon - self.previousGPSlocation._lon)
            vitesseEstX = groundSpeed * cos(angle)
            vitesseNordY = groundSpeed * sin(angle)
        self.previousGPSlocation = self.vehicle.location
        return vitesseEstX, vitesseNordY



    # Fonction prenant en entrée les coordonnées en x et y, en pixels, de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage_fonctionnel(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            print("Consigne nulle")
            #self.set_velocity(0, 0, 0)
            return None, None, None, None

        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance        
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        self.kp_atterrissage = 0.005 if altitude < 5 else 0.008
        self.kp_atterrissage *= self.coefficient_kp_atterrissage

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX_2 = self.camera.x_imageCenter - aruco_center_x
        erreurY_2 = self.camera.y_imageCenter - aruco_center_y + self.offset_camera_atterrissage
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX_2**2+erreurY_2**2)
        dist_angle = atan2(erreurY_2, erreurX_2)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurX = dist_center * cos(alpha)
        erreurY = dist_center * sin(alpha)
        # Si l'erreur selon x et y est inférieure à 25 cm, on la considère comme nulle
        if abs(erreurX) <= 10:  
            erreurX = 0
        if abs(erreurY) <= 10:
            erreurY = 0

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeX = (erreurX - self.erreurAnterieureX_atterrissage)
        erreurDeriveeY = (erreurY - self.erreurAnterieureY_atterrissage)
        # Erreur intégrale
        self.erreurIntegraleX_atterrissage += erreurX
        self.erreurIntegraleY_atterrissage += erreurY
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureX_atterrissage = erreurX
        self.erreurAnterieureY_atterrissage = erreurY

        # Calcul de la vitesse corrigée 
        vx = self.kp_atterrissage * erreurX + self.kd_atterrissage * erreurDeriveeX + self.ki_atterrissage * self.erreurIntegraleX_atterrissage
        vy = self.kp_atterrissage * erreurY + self.kd_atterrissage * erreurDeriveeY + self.ki_atterrissage * self.erreurIntegraleY_atterrissage        
        # Correction en fonction de l'angle 
        #vx *= abs(cos(self.vehicle.attitude.roll) ** self.coef_vx_atterrissage)
        #vy *= abs(cos(self.vehicle.attitude.pitch) ** self.coef_vy_atterrissage)
        # Bornage des vitesses à +/- 5 m/s
        vx = -min(max(vx, -5.0), 5.0)
        vy = min(max(vy, -5.0), 5.0)
        
        # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus du drone 
        dist_center_threshold = 50 if altitude < 2 else 1000        
        # Si n'est dans un rayon d'un mètre autour du drone, il ne change pas son altitude 
        if dist_center > dist_center_threshold :
            vz = 0
        # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
        else:
        #Choix de la vitesse verticale en fonction de l'altitude
            if altitude > 8:
                vz = 1.5 
            elif altitude > 5:
                vz = 1
            elif altitude > 1.5:
                vz = 0.5
            else:
                vz = 0
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : VX = " + str(vx) + " ; VY = " + str(vy) + " ; VZ = " + str(vz))
        self.set_velocity(vy, vx, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
        return erreurX_2, erreurY_2, vx, vy
    


    # Fonction prenant en entrée les coordonnées en x et y, en METRES, de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage_metres(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            return None, None, None, None

        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = (self.camera.x_imageCenter - aruco_center_x)/(self.camera.x_imageCenter) * altitude * tan(radians(self.camera.horizontal_field_view/2))
        erreurY = (self.camera.y_imageCenter - aruco_center_y)/(self.camera.y_imageCenter) * altitude * tan(radians(self.camera.vertical_field_view/2)) + 0.2
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurEst = dist_center * cos(alpha)
        erreurNord = dist_center * sin(alpha)
        # Si l'erreur selon x et y est inférieure à 25 cm, on la considère comme nulle
        if abs(erreurEst) <= 0.25:  
            erreurEst = 0
        if abs(erreurNord) <= 0.25:
            erreurNord = 0

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeX = (erreurEst - self.erreurAnterieureX_atterrissage)
        erreurDeriveeY = (erreurNord - self.erreurAnterieureY_atterrissage)
        # Erreur intégrale
        self.erreurIntegraleX_atterrissage += erreurEst
        self.erreurIntegraleY_atterrissage += erreurNord
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureX_atterrissage = erreurEst
        self.erreurAnterieureY_atterrissage = erreurNord

        # Calcul de la vitesse corrigée 
        VEst = self.kp_atterrissage * erreurEst + self.kd_atterrissage * erreurDeriveeX + self.ki_atterrissage * self.erreurIntegraleX_atterrissage
        VNord = self.kp_atterrissage * erreurNord + self.kd_atterrissage * erreurDeriveeY + self.ki_atterrissage * self.erreurIntegraleY_atterrissage        
        # Bornage des vitesses à +/- 5 m/s
        VEst = -min(max(VEst, -5.0), 5.0)
        VNord = min(max(VNord, -5.0), 5.0)
        return erreurX, erreurY, erreurNord, erreurEst, VEst, VNord                

        
#-------------------------------------------------------------------------------------------------------------------------------------------
# Atterissage Aruco
# ------------------------------------------------------------------------------------------------------------------------------------------
        
    def atterrissage_aruco_fonctionnel(self, chemin_dossier):
        
        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        # Début du chronomètre
        start_time = time.time()
        
        # Tant que le drone n'est pas à 2 m du sol ou que le temps écoulé est inférieur à 30 secondes, 
        # on lance l'asservissement du drone
        while altitude > 1.5 and (start_time-time.time()) < 30000:
            
            # Récupération de l'altitude du drone
            altitude = self.vehicle.rangefinder.distance
            
            # Si le robot est à plus de 10 mètres du sol on le fait descendre
            if altitude > 7.5:
                print("Descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol
                continue
                        
            # Si le robot est à moins de 7.5 mètres on détecte directement l'aruco et on récupère les coordonnées de son centre            
            else:  
                centre_aruco_X, centre_aruco_Y, _, image = self.camera.detection_aruco(True)
                # Asservissement par rapport au centre de l'aruco
                erreurX, erreurY, vx, vy = self.asservissement_atterrissage_fonctionnel(centre_aruco_X, centre_aruco_Y)
                # Affichage de l'erreur et de la vitesse
                image = cv2.putText(image, "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
                image = cv2.putText(image, "Vitesse : Vx = " + str(vx) + " ; Vy = " + str(vy), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
                # Traçage d'un cercle au centre de l'image
                cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
                # Sauvegarde de la photo
                enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")
            
            
        # Une fois que le robot est assez bas, on le fait atterrir
        print("Atterrissage")
        self.set_mode("LAND")




    def atterrissage_aruco_test(self, chemin_dossier):

        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        # Début du chronomètre
        start_time = time.time()

        while altitude > 2 or (start_time-time.time()) < 30000 :
            
            # Récupération de l'altitude du drone
            altitude = self.vehicle.rangefinder.distance
            
            # Si le robot est à plus de 10 mètres du sol on le fait descendre
            if altitude > 7.5:
                print("Drone trop haut : descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol
                continue
                      
            #on détecte l'aruco et on récupère les coordonnées de son centre            
            centre_aruco_X, centre_aruco_Y, _, image = self.camera.detection_aruco(True)
            # Si le drone ne détecte pas d'aruco, on le fait voler plus haut et on enregistre la photo
            if centre_aruco_X == None:
                print("Aruco non détecté : montée du drone")
                self.set_velocity(0, 0, -1) #sens z positif -> vers le sol
                # Sauvegarde de la photo
                enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")
                continue         

            # Asservissement par rapport au centre de l'aruco
            erreurX, erreurY, vx, vy = self.asservissement_atterrissage_fonctionnel(centre_aruco_X, centre_aruco_Y+140)
            # Affichage de l'erreur et de la vitesse
            image = cv2.putText(image, "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
            image = cv2.putText(image, "Vitesse : Vx = " + str(vx) + " ; Vy = " + str(vy), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
            # Traçage d'un cercle au centre de l'image
            cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
            # Sauvegarde de la photo
            enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")

            # Détermination de la vitesse verticale
            if altitude < 1.5 :
                vz = 0
            elif altitude < 5:
                vz = 0.5
            elif altitude > 9:
                vz = 1

            #Envoie de la consigne de vitesse au drone
            print("Consigne en vitesse : VX = " + str(vx) + " ; VY = " + str(vy) + " ; VZ = " + str(vz))
            self.set_velocity(vy, vx, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North' 
            break

                    
        # Une fois que le robot est assez bas, on le fait atterrir
        print("Atterrissage")
        self.set_mode("LAND")


        
  
    def atterrissage_aruco_david(self):
        
        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        
        # Tant que le drone n'est pas à 50 cm du sol, on lance l'asservissement du drone
        while altitude > 7:
                print("Descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol
                altitude = self.vehicle.rangefinder.distance
                
        # Détection du centre de l'aruco
        centre_aruco_X, centre_aruco_Y, _ = self.camera.detection_aruco()
            
        # Estimating marker location from vision
        estimated_location = get_GPS_through_picture(self, centre_aruco_X, centre_aruco_Y)
        print("Déplacement jusqu'à la position de l'aruco")
        self.vehicle.goto(estimated_location, 0.25)

        # Envoi de la commande d'atterissage
        print("Atterissage sur aruco")
        msg = self.vehicle.message_factory.landing_target_encode(
            0,          # time_boot_ms (non utilisé)
            0,          # target num
            0,          # frame
            estimated_location.lat, # X 
            estimated_location.lon, # Y 
            0,          # altitude. Not supported.
            0,0         # size of target in radians
        )
        self.vehicle.send_mavlink(msg)
        #self.set_mode("LAND")
        self.vehicle.flush()