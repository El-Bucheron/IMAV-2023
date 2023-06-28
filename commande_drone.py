#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""
import time
from math import atan2, cos, sin, sqrt
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
from utilities import get_distance_metres
from detection_target import Detection


class Drone:
    
    # Constructeur de la classe se connectant au drone
    def __init__(self):     
        
        # Coefficients de l'asservissement PID de l'atterrissage
        self.kp_atterrissage = 0 # Coefficient de la correction proportionnelle mis à 0 car initialisé plus tard
        self.kd_atterrissage = 0.0001  # 0.00001 working "fine" for both
        self.ki_atterrissage = 0.000001  # 0.0000001
        
        # Coefficients de l'asservissement PID du suivi de véhicule
        self.kp_suivi_vehicule = 0.005 
        self.kd_suivi_vehicule = 0.0001  # 0.00001 working "fine" for both
        self.ki_suivi_vehicule = 0.000001  # 0.0000001

        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX = 0
        self.erreurIntegraleY = 0
        self.erreurAnterieureX = 0
        self.erreurAnterieureY = 0
        
        # Connexion au drone et initialisation de la caméra
        print("Connexion au drone et initialisation de la caméra")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=2)
        self.camera = Detection()
        print("Connexion et initialisation terminées")




    #set_mode - set the mode of the vehicle as long as we are in control
    def set_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        print("[mission] Mode set to %s." % mode)
        self.vehicle.flush()
            
    #get_mode - get current mode of vehicle 
    def get_mode(self):
        return self.vehicle.mode.name
    



    # Décollage du drone jusqu'à la distance fournie en argument
    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)
                  

    
    
    # Définition de la consigne de vitesse selon le repère x,y,z du drone et pendant une durée de 0.1 seconde  
    def set_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        # only let commands through at 10hz
        print("[mission] Velocity set to values: vx: %.2f ; vy: %.2f ; vz %.2f." % (velocity_x, velocity_y, velocity_z))

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

        self.vehicle.send_mavlink(msg)
        time.sleep(0.1)

                       
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
            time.sleep(1)




    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_suivi_vehicule(self):

        # Récupération du centre l'aruco
        aruco_center_x, aruco_center_y = self.camera.detection_aruco_2023()
        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            print("Aruco non détecté")
            return
        
        # Si l'aruco a été détecté, on affiche ses coordonnées et on continue l'asservissement
        print("Aruco trouvé de centre X = " + str(aruco_center_x) + " ; Y = " + str(aruco_center_y)) 


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
        # Si l'erreur selon x et y est inférieure à 10 pixel, on la considère comme nulle
        if abs(erreurX) <= 10:  
            erreurX = 0
        if abs(erreurY) <= 10:
            erreurY = 0

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        self.erreurDeriveeX = (erreurX - self.erreurAnterieureX)
        self.erreurDeriveeY = (erreurY - self.erreurAnterieureY)
        # Erreur intégrale
        self.erreurIntegraleX += erreurX
        self.erreurIntegraleY += erreurY
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureX = erreurX
        self.erreurAnterieureY = erreurY

        # Calcul de la vitesse corrigée 
        vx = self.kp_suivi_vehicule * erreurX + self.kd_suivi_vehicule * self.erreurDeriveeX + self.ki_suivi_vehicule * self.erreurIntegraleX
        vy = self.kp_suivi_vehicule * erreurY + self.kd_suivi_vehicule * self.erreurDeriveeY + self.ki_suivi_vehicule * self.erreurIntegraleY        
        # Bornage des vitesses à +/- 15 m/s
        vx = -min(max(vx, -15.0), 15.0)
        vy = min(max(vy, -15.0), 15.0)
        
        #Envoie de la consigne de vitesse au drone
        self.drone.set_velocity(vy, vx, 0) # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'


#-------------------------------------------------------------------------------------------------------------------------------------------
# Atterissage Aruco
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            print("Aruco non détecté")
            return

        # Récupération de l'altitude du drone
        altitudeAuSol = self.vehicle.rangefinder.distance        
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        self.kp_atterrissage = 0.003 if altitudeAuSol < 5 else 0.005

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = self.camera.x_imageCenter - aruco_center_x
        erreurY = self.camera.y_imageCenter - aruco_center_y
        print("Erreur en X = " + str(erreurX) + " ; Erreur en Y = " + str(erreurY))
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurX = dist_center * cos(alpha)
        erreurY = dist_center * sin(alpha)
        # Si l'erreur selon x et y est inférieure à 10 pixel, on la considère comme nulle
        if abs(erreurX) <= 10:  
            erreurX = 0
        if abs(erreurY) <= 10:
            erreurY = 0

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        self.erreurDeriveeX = (erreurX - self.erreurAnterieureX)
        self.erreurDeriveeY = (erreurY - self.erreurAnterieureY)
        # Erreur intégrale
        self.erreurIntegraleX += erreurX
        self.erreurIntegraleY += erreurY
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureX = erreurX
        self.erreurAnterieureY = erreurY

        # Calcul de la vitesse corrigée 
        vx = self.kp_atterrissage * erreurX + self.kd_atterrissage * self.erreurDeriveeX + self.ki_atterrissage * self.erreurIntegraleX
        vy = self.kp_atterrissage * erreurY + self.kd_atterrissage * self.erreurDeriveeY + self.ki_atterrissage * self.erreurIntegraleY        
        # Bornage des vitesses à +/- 5 m/s
        vx = -min(max(vx, -5.0), 5.0)
        vy = min(max(vy, -5.0), 5.0)
        
        # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus du drone 
        dist_center_threshold = 50 if altitudeAuSol < 2 else 1000        
        # Si le drone n'est pas assez proche, il reste dans le même plan
        if dist_center > dist_center_threshold :
            vz = 0
        # Si le drone est assez proche, on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
        else:
            #Choix de la vitesse verticale en fonction de l'altitude
            if altitudeAuSol < 3 :
                vz = 0.1  # a changer pour descendre
            elif altitudeAuSol > 9 :
                vz = 1  # a changer pour descendre
            elif altitudeAuSol > 5:
                vz = 0.5
            else:
                vz = 0.25
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : VX = " + vx + " ; VY = " + vy + " ; VZ = " + vz )
        self.drone.set_velocity(vy, vx, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'

        
        
        
    def atterrissage_aruco(self):
        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        # Tant que le drone n'est pas à 25 cm du sol, on lance l'asservissement du drone
        while altitude > 0.25:
            # Si le robot est à plus de 10 mètres (5 pour le test) du sol on le fait descendre
            if altitude > 5:
                print("Descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol

            
            # Si le robot est entre 5 et 10 mètres du sol on cherche l'aruco par détection de carré blanc
            # On récupère ensuite le centre de l'aruco détecté selon X et Y (en pixel)
            #elif altitude > 5:
                #centre_aruco_X, centre_aruco_Y = self.camera.detection_carre_blanc()
            

            # Si le robot est à moins de 5 mètres on détecte directement l'aruco et on récupère les coordonnées de son centre
            else:
                centre_aruco_X, centre_aruco_Y = self.camera.detection_aruco_2023()
            print("Coordonnées trouvées : x = " + str(centre_aruco_X) + " ; y : " + str(centre_aruco_Y))
            # On asservit le drone avec pour consigne la position du centre l'aruco 
            self.asservissement_atterrissage(centre_aruco_X, centre_aruco_Y)
            # On récupère l'altitude pour entamer la nouvelle boucle d'asservissement
            altitude = self.vehicle.rangefinder.distance
        # Une fois que le robot est assez bas, on le fait atterrir
        print("Atterrissage")
        self.set_mode("LAND")