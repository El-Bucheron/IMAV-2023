#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""

import numpy as np
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
        #self.kp_atterrissage = 0 # Coefficient mis à 0 car initialisé plus tard
        #self.kd_atterrissage = 0.0001  # 0.00001 working "fine" for both
        #self.ki_atterrissage = 0.000001  # 0.0000001

    # Coefficients de l'asservissement PID de l'atterrissage pour l'erreur en METRES
        self.kp_atterrissage = 0.05 # Coefficient mis à 0 car initialisé plus tard
        self.kd_atterrissage = 0.001  # 0.00001 working "fine" for both
        self.ki_atterrissage = 0.00001  # 0.0000001

        # Coefficients de l'asservissement PID du suivi de véhicule
        self.kp_suivi_vehicule = 0.005 
        self.kd_suivi_vehicule = 0.0001  # 0.00001 working "fine" for both
        self.ki_suivi_vehicule = 0.000001  # 0.0000001

        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX_atterrissage = 0
        self.erreurIntegraleY_atterrissage = 0
        self.erreurAnterieureX_atterrissage = 0
        self.erreurAnterieureY_atterrissage = 0
        
        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX_suivi_vehicule = 0
        self.erreurIntegraleY_suivi_vehicule = 0
        self.erreurAnterieureX_suivi_vehicule = 0
        self.erreurAnterieureY_suivi_vehicule = 0
        
        # Connexion au drone et initialisation de la caméra
        print("Connexion au drone et initialisation de la caméra")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=2)
        self.camera = Detection()
        print("Connexion et initialisation terminées")




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



    def goto(self, targetLocation, distanceAccuracy):
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
        """
        Function to move to a target location with a given precision.

        Based on the simple_goto function from DroneKit completed with a
        wait function checking if the drone is in a desired accuracy
        circle around the target location.
        """
        # Simple goto DroneKit function
        self.vehicle.simple_goto(targetLocation)

        # Stop action if we are no longer in GUIDED mode
        while self.vehicle.mode.name=="GUIDED": 
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
# Suivi de véhicule
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_suivi_vehicule(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            print("Consigne nulle")
            return
        
        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = self.camera.x_imageCenter - aruco_center_x
        erreurY = self.camera.y_imageCenter - aruco_center_y
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
        # Bornage des vitesses à +/- 17.5 m/s
        vx = -min(max(vx, -17.5), 17.5)
        vy = min(max(vy, -17.5), 17.5)
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : VX = " + str(vx) + " ; VY = " + str(vy))
        self.set_velocity(vy, vx, 0) # Pour le sense de la camera, X pointe vers l'est et Y vers le nord




#-------------------------------------------------------------------------------------------------------------------------------------------
# Atterissage Aruco
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction prenant en entrée les coordonnées en x et y, en pixels, de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            print("Consigne nulle")
            return

        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance        
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        #self.kp_atterrissage = 0.003 if altitude < 5 else 0.005

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        #erreurX = self.camera.x_imageCenter - aruco_center_x
        #erreurY = self.camera.y_imageCenter - aruco_center_y
        # Distance en mètres entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = (self.camera.x_imageCenter - aruco_center_x)/(self.camera.x_imageCenter) * altitude * tan(radians(self.camera.horizontal_field_view/2)) 
        erreurY = (self.camera.y_imageCenter - aruco_center_y)/(self.camera.y_imageCenter) * altitude * tan(radians(self.camera.vertical_field_view/2))
        print("Erreur en mètres : EX = " + str(erreurX) + " ; EY = " + str(erreurY))
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurX = dist_center * cos(alpha)
        erreurY = dist_center * sin(alpha)
        # Si l'erreur selon x et y est inférieure à 25 cm, on la considère comme nulle
        if abs(erreurX) <= 0.25: #10:  
            erreurX = 0
        if abs(erreurY) <= 0.25: #10:
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
        # Bornage des vitesses à +/- 5 m/s
        vx = -min(max(vx, -5.0), 5.0)
        vy = min(max(vy, -5.0), 5.0)
        
        # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus du drone 
        #dist_center_threshold = 50 if altitude < 2 else 1000        
        # Si n'est dans un rayon d'un mètre autour du drone, il ne change pas son altitude 
        #if dist_center > dist_center_threshold :
        #    vz = 0
        # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
        #else:
        #Choix de la vitesse verticale en fonction de l'altitude
        if altitude > 9:
            vz = 1.5 #1
        elif altitude > 5:
            vz = 1 #0.5
        elif altitude > 3:
            vz = 0.5 #0.25
        else:
            vz = 0.4 #0.2
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : VX = " + str(vx) + " ; VY = " + str(vy) + " ; VZ = " + str(vz))
        self.set_velocity(vy, vx, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'

        
        
        
    def atterrissage_aruco_matthieu(self, chemin_dossier = ""):
        
        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        
        # Tant que le drone n'est pas à 50 cm du sol, on lance l'asservissement du drone
        while altitude > 0.5:
            
            # Récupération de l'altitude du drone
            altitude = self.vehicle.rangefinder.distance
            
            # Si le robot est à plus de 10 mètres du sol on le fait descendre
            if altitude > 15:
                print("Descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol
                continue
                
                
            # Si le robot est entre 15 et 75 mètres du sol on cherche l'aruco par détection de carré blanc
            # On récupère ensuite le centre de l'aruco détecté selon X et Y (en pixel)
            elif altitude > 7.5:
                print("Détection par carré blanc")
                centre_aruco_X, centre_aruco_Y, image, image_filtree = self.camera.detection_carre_blanc(altitude, True)
                
                if chemin_dossier != "":
                    
                    # Traçage d'un cercle au centre de l'image
                    cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
                    # Sauvegarde de la photo
                    enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")
                    enregistrement_photo_date_position(self, image_filtree, chemin_dossier, ("yes" if centre_aruco_X != None else "no") + " filtre")

                      
            # Si le robot est à moins de 5 mètres on détecte directement l'aruco et on récupère les coordonnées de son centre            
            else:
                
                print("Détection par aruco")
                centre_aruco_X, centre_aruco_Y, _, image = self.camera.detection_aruco(True)

                if chemin_dossier != "":
                    # Traçage d'un cercle au centre de l'image
                    cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
                    # Sauvegarde de la photo
                    enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")


            # On asservit le drone avec pour consigne la position du centre l'aruco    
            print("Coordonnées trouvées : x = " + str(centre_aruco_X) + " ; y = " + str(centre_aruco_Y)) 
            self.asservissement_atterrissage(centre_aruco_X, centre_aruco_Y)
            
            
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
        distance_vision, angle_vision = get_distance_angle_picture(self.camera.x_imageCenter, self.camera.y_imageCenter,
                                                                 centre_aruco_X, centre_aruco_Y,
                                                                 self.vehicle.rangefinder.distance, self.camera.dist_coeff_x, self.camera.dist_coeff_y)
        current_location = LocationGlobalRelative(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, 0)
        estimated_location = get_GPS_location(current_location, self.vehicle.attitude.yaw + angle_vision, distance_vision)
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
