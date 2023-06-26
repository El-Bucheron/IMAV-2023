# Code permettant d'importer les classes "Drone" et "Detection"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from commande_drone import Drone
from detection_target import Detection

import cv2
import cv2.aruco as aruco

from time import sleep
from math import atan2, cos, sin, sqrt

# Boolean variables
aruco_seen = None
good_aruco_found = None
white_square_seen = None

# Counter variables
counter_no_detect = None
counter_white_square = None

x_centerPixel_target = None
y_centerPixel_target = None
altitudeAuSol = None
id_to_test = None
altitudeRelative = None

drone = Drone()
camera = Detection()

def asservissement(drone, camera, last_errx, last_erry, errsumx, errsumy):

    # PID Coefficients
    kpx = 0.005
    kpy = 0.005
    kdx = 0.0001  # 0.00001 working "fine" for both
    kdy = 0.0001
    kix = 0.000001  # 0.0000001
    kiy = 0.000001

    vx = 0
    vy = 0
    vz = 0

    if altitudeAuSol < 5 :
        kpx = 0.003
        kpy = 0.003
    else :
        kpx = 0.005
        kpy = 0.005

    print("Pixels values - x:%s - y:%s" % (x_centerPixel_target, y_centerPixel_target))
    if x_centerPixel_target == None or y_centerPixel_target == None :   # echec Detection
        if counter_no_detect > 10 :   #on fixe le nombre d'image consecutive sans Detection pour considerer qu il ne detecte pas
            if altitudeAuSol > 30 :  # si on altitudeRelative sup a 25m stop le thread
                drone.set_velocity(0, 0, 0, 1)
                print ("[asserv] altitudeRelative > 30")
                return 0, 0, 0, 0
            else :  # pas de Detection Drone prend de l altitude
                vx = 0
                vy = 0
                vz = 0
                drone.set_velocity(vx, vy, vz, 1)
            #
        elif counter_no_detect > 1 :   # fixer la position du Drone en cas de non Detection
            drone.set_velocity(0, 0, 0, 1)
            #print ("[asserv] compteur_no_detect > 2   stabilisation drone")

        errx = 0
        erry = 0
        errsumx = 0
        errsumy = 0
    
    else :  # Detection ok
        print ("[asserv] detection OK")
        #Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        errx = camera.x_imageCenter - x_centerPixel_target
        erry = camera.y_imageCenter - y_centerPixel_target
        #Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = math.sqrt(errx**2+erry**2)
        dist_angle = atan2(erry, errx)
        #Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + drone.vehicle.attitude.yaw
        errx = dist_center * cos(alpha)
        erry = dist_center * sin(alpha)
        #Si l'erreur selon x et y est inférieure à 10 pixel, on la considère comme nulle
        if abs(errx) <= 10:  
            errx = 0
        if abs(erry) <= 10:
            erry = 0

        # PD control
        dErrx = (errx - last_errx)# / delta_time
        dErry = (erry - last_erry)# / delta_time
        errsumx += errx# * delta_time
        errsumy += erry# * delta_time

        #Calcul de la vitesse corrigée
        vx = (kpx * errx) + (kdx * dErrx) + (kix * errsumx)
        vy = (kpy * erry) + (kdy * dErry) + (kiy * errsumy)

        #Choix de la vitesse verticale en fonction de l'altitude
        if altitudeAuSol < 3 :
            vz = 0.1  # a changer pour descendre
        elif altitudeAuSol > 9 :
            vz = 1  # a changer pour descendre
        elif altitudeAuSol > 5:
            vz = 0.5
        else:
            vz = 0.25

        # Establish limit to outputs
        vx = min(max(vx, -5.0), 5.0)
        vy = min(max(vy, -5.0), 5.0)
        vx = -vx                        # High opencv is south Dronekit

        # Dronekit
        # X positive Forward / North
        # Y positive Right / East
        # Z positive down

        if altitudeAuSol < 2 :
            dist_center_threshold = 50

        else :
            dist_center_threshold = 1000

        if dist_center <= dist_center_threshold :
            drone.set_velocity(vy, vx, vz, 1) 

        else :
            #lancer un deplacement pour ce rapprocher du centre sans descendre ou monter
            vz = 0
            drone.set_velocity(vy, vx, vz, 1)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'

    # Return last errors and sums for derivative and integrator terms
    return errx, erry, errsumx, errsumy



#--------------------------------------------------------------
def mission_largage_zone_inconnu(id_to_find):

    id_to_test = -1

    last_errx = 0
    last_erry = 0
    errsumx = 0
    errsumy = 0

    while drone.get_mode() != "GUIDED":
        print("En attente du mode auto")
        sleep(1)


    while (drone.get_mode() == "GUIDED" or drone.get_mode() == "AUTO") or not package_dropped:
        # actualisation de l'altitude et des coordonnées gps
        altitudeAuSol = drone.vehicle.rangefinder.distance
        altitudeRelative = drone.vehicle.location.global_relative_frame.alt
        longitude = drone.vehicle.location.global_relative_frame.lon
        latitude = drone.vehicle.location.global_relative_frame.lat
        heading = drone.vehicle.attitude.yaw
    
        #le srcipt Detection Target
        x_centerPixel_target, y_centerPixel_target, aruco_seen, good_aruco_found, white_square_seen, saved_markers = camera.Detection_aruco(latitude, longitude, altitudeAuSol, heading, saved_markers, id_to_test, True)
        # Asservissement control
        if drone.get_mode() == "GUIDED" :
            print ("Condition GUIDED OK")
            last_errx, last_erry, errsumx, errsumy = asservissement(drone, camera, last_errx, last_erry, errsumx, errsumy)
    
        if drone.get_mode() != "GUIDED" and drone.get_mode() != "AUTO":
            break
  
        #--------------- Case 1: Good ArUco ID found -----------------------
        if good_aruco_found:
            print("[detection] Case 1: Good ArUco ID found!")
            counter_no_detect = 0

            while drone.get_mode() != "GUIDED":
                drone.set_mode("GUIDED")

            dist_center = math.sqrt((camera.x_imageCenter-x_centerPixel_target)**2+(camera.y_imageCenter-y_centerPixel_target)**2)
            print("[mission] Current distance: %.2fpx ; Altitude: %.2fm." % (dist_center, altitudeAuSol))

            if dist_center <= 75 and altitudeAuSol < 1.5:  # condition pour faire le largage
                print("[mission] Largage !")
                drone.move_servo(10, True)
                time.sleep(0.5)
                package_dropped = True
                break

        #--------------- Case 2: Some white square seen --------------------
        elif white_square_seen:
            print("[detection] Case 2: Some white square seen.")

            counter_no_detect = 0
            counter_white_square += 1

            print("[mission] Detection of 1 or many white squares (%i times)" % counter_white_square)

            # Check saved_ids in detection dictionary
            for saved_id in saved_markers :
                # Check boolean: if False, needs to be explored
                if saved_markers[saved_id][1] == False:
                  # if saved_id > 1001 and saved_markers[saved_id-1][1] == False:
                  #  saved_markers[saved_id-1].pop()
                    while drone.get_mode() != "GUIDED":
                        drone.set_mode("GUIDED")
                    id_to_test = saved_id
                    print("[mission] Detection targetted towards id %s" % id_to_test)

        #--------------- Case 3: ArUco tag seen but id false ---------------
        elif aruco_seen:
            print("[detection] Case 3: ArUco seen BUT not good ArUco found.")

            counter_no_detect = 0

            # Since it is not the good ArUco, continue the AUTO mission
            while drone.get_mode() != "AUTO" :
                drone.passage_mode_Auto()

            # Reset visual PID errors
            last_errx = 0
            last_erry = 0
            errsumx = 0
            errsumy = 0

            saved_markers[id_to_test] = (saved_markers[id_to_test][0], True)
            id_to_test = -1

        #--------------- Case 4: No detection of white or ArUco ------------
        else:
            print("[detection] Case 4: No detection of white or ArUco.")
            counter_no_detect += 1
            counter_white_square = 0
      
            if counter_no_detect > 5:
                print("[mission] 5 times without tag or white detection, not interesting place.")

                while drone.get_mode() != "AUTO" :
                    drone.passage_mode_Auto()

                    # Reset visual PID errors
                    last_errx = 0
                    last_erry = 0
                    errsumx = 0
                    errsumy = 0

                    saved_markers[id_to_test] = (saved_markers[id_to_test][0], True)
                    id_to_test = -1

    if drone.get_mode() == "GUIDED" or drone.get_mode() == "AUTO":  #securite pour ne pas que le drone reprenne la main en cas d interruption
    #########repart en mode RTL
        drone.set_mode("RTL") #### modif preciser qu on est en guided avant et ajouter l altitude du RTL