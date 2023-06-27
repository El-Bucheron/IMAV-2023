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
        
        # Coefficients de l'asservissement PID
        self.kpx = 0 # Coefficient de la correction proportionnelle mis à 0 car il sera initialisé plus tard
        self.kpy = 0
        self.kdx = 0.0001  # 0.00001 working "fine" for both
        self.kdy = 0.0001
        self.kix = 0.000001  # 0.0000001
        self.kiy = 0.000001
        
        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleX = 0
        self.erreurIntegraleY = 0
        self.erreurAnterieureX = 0
        self.erreurAnterieureY = 0
        
        # Connexion au drone et initialisation de la caméra
        print("Connexion au drone et initialisation de la caméra")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=2)
        self.camera = Detection()
        print("Connexion et initialisation terminée")



        
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
            
         

        
    def passage_mode_Auto(self):
        """
        Permet d'initialiser le code pour lancer la mission en auto
        """
        print("[mission] Starting mission AUTO.")
        # Reset mission set to first (0) waypoint
        self.vehicle.commands.next=0

        # Set mode to AUTO to start mission
        self.vehicle.mode = VehicleMode("AUTO")
           

            
            
            
    #Fonction servant à faire décoller le drone après passage en mode "AUTO"
    def lancement_decollage(self, altitudeDeVol):
        #Initialisaion du programme en mode stabilize
        self.vehicle.mode = VehicleMode("STABILIZE")
        # Attente du mode auto
        while self.get_mode() != "AUTO":
            print("En attente du mode AUTO")
            time.sleep(1)
        #décollage
        self.arm_and_takeoff(altitudeDeVol)       
            
            
    #set_mode - set the mode of the vehicle as long as we are in control
    def set_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        print("[mission] Mode set to %s." % mode)
        self.vehicle.flush()
            
    #get_mode - get current mode of vehicle 
    def get_mode(self):
        last_mode = self.vehicle.mode.name
        return last_mode

    
    
    
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

        # send command to vehicle
        for x in range(0, duration) :
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


            
            
    def readmission(self,aFileName):
        """
        Load a mission from a file into a list. The mission definition is in the Waypoint file
        format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

        This function is used by upload_mission().
        """
        print("\nReading mission from file: %s" % aFileName)
        cmds = self.vehicle.commands
        missionlist=[]
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist
    
    def upload_mission(self, aFileName):
        """
        Upload a mission from a file. 
        """
        #Read mission from file
        missionlist = self.readmission(aFileName)

        print("\nUpload mission from a file: %s" % aFileName)
        #Clear existing mission from vehicle
        print(' Clear mission')
        cmds = self.vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print(' Upload mission')
        self.vehicle.commands.upload()

    def download_mission(self):
        """
        Downloads the current mission and returns it in a list.
        It is used in save_mission() to get the file information to save.
        """
        print(" Download mission from vehicle")
        missionlist=[]
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        for cmd in cmds:
            missionlist.append(cmd)
        return missionlist

    def save_mission(self, aFileName):
        """
        Save a mission in the Waypoint file format 
        (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
        """
        print("\nSave mission from Vehicle to file: %s" % aFileName)    
        #Download mission from vehicle
        missionlist = self.download_mission()
        #Add file-format information
        output='QGC WPL 110\n'
        #Add home location as 0th waypoint
        home = self.vehicle.home_location
        print("home.lat : "+str(home.lat))
        output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
        #Add commands
        for cmd in missionlist:
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            output+=commandline
        with open(aFileName, 'w') as file_:
            print(" Write mission to file")
            file_.write(output)
        
        
        
        
    def printfile(self, aFileName):
        """
        Print a mission file to demonstrate "round trip"
        """
        print("\nMission file: %s" % aFileName)
        with open(aFileName) as f:
            for line in f:
                print(' %s' % line.strip()) 


                
                
    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=self.vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint



    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage(self, aruco_center_x, aruco_center_y):

        # Récupération de l'altitude du drone
        altitudeAuSol = self.vehicle.rangefinder.distance        
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        self.kpx = 0.003 if altitudeAuSol < 5 else 0.005
        self.kpy = self.kpx

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

        # PD control
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
        vx = self.kpx * erreurX + self.kdx * self.erreurDeriveeX + self.kix * self.erreurIntegraleX
        vy = self.kpy * erreurY + self.kdy * self.erreurDeriveeY + self.kiy * self.erreurIntegraleY        
        # Bornage des vitesses à +/- 5 m/s
        vx = min(max(vx, -5.0), 5.0)
        vy = min(max(vy, -5.0), 5.0)
        vx = -vx # Inversion du sens de la vitesse pour correpondre au sens 
        
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
        self.drone.set_velocity(vy, vx, vz, 1)

        
        
        
    def atterrissage_aruco(self):
        print("")