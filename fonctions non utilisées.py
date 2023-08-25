# Classe Drone

"""
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
"""


# Classe Detection

"""
    # Fonction servant à détecter l'aruco quand l'altitude est trop élevée pour qu'on le détecte directement avec la fonction "detection_aruco"
    # Pour cela, on assume que l'aruco vu du ciel, après quelques corrections d'images, est un carré blanc
    # La fonction renvoie les coordonnées du carré blanc si elle en trouve un ou des variables vides sinon
    # On mettant le paramètre "return_image" à True, on renvoie également l'image acquise avec visualisation du contour et son centre
    # Si aucun carré blanc n'a pas été détecté et que "return_image=True", on renvoie l'image filtrée    
    def detection_carre_blanc(self, altitude, return_image = False):
        
        # Prise de la photo avec la PiCamera
        image = self.prise_photo()
        # Application d'un floutage gaussion
        blur = cv2.GaussianBlur(image,(5,5),0)
        # Conversion de l'image floutée en HLS
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS) 
        # Application du masque
        mask_hls = cv2.inRange(hls, self.lower_bound_filtre_blanc, self.upper_bound_filtre_blanc)
        # Masque permettant de remplir le centre d'un contour avec une unique couleur
        mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, self.closing_kernel)
        # Détection des contours
        contours, _ = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # On regarde parmis les contours s'il n'y a pas un carré blanc
        for c in contours:
            # ?
            peri = cv2.arcLength(c, True)
            # Récupération des segments composant le contour
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            # Récupération de l'aire du contour
            area = cv2.contourArea(c)
            # Récupération de la longueur et largeur du contour
            (x, y, w, h) = cv2.boundingRect(approx)

            # On vérifie que le contour possède la bonne aire
            # que le rapport longueur/largeur est compris entre 0.9 et 1.1 
            # et que le contour possède 4 côtés
            if 10000*altitude**-2 < area < 60000*altitude**-2 and 0.95 <= (w / float(h)) <= 1.05 and len(approx) == 4:

                # Calcul du centre du carré
                M = cv2.moments(c)
                x_aruco_center = int(M["m10"] / M["m00"])
                y_aruco_center = int(M["m01"] / M["m00"])

                # On vérifie que le centre du carré est blanc
                if mask_closing[y_aruco_center,x_aruco_center] == 255:

                    # Si on ne souhaite pas renvoyer l'image, on renvoie uniquement les coordonnées du centre du carré blanc détecté
                    if return_image == False:
                        return x_aruco_center, y_aruco_center
                    
                    # Si l'on souhaite renvoyer l'image, on trace les éléments graphiques permettant de montrer que la détection a bien été réalisée
                    # On trace le contour détecté
                    cv2.drawContours(image, [c], 0, (0,255,0), 3)
                    # On trace un point rouge au centre du contour pour vérifier quel point est regardé
                    cv2.circle(image, (x_aruco_center, y_aruco_center), 2, (0, 0, 255), -1)
                    # On renvoie les coordronnées calculées et l'image modifiée
                    return x_aruco_center, y_aruco_center, image, mask_closing


        # Si aucun carré blanc n'a pas été détecté, on renvoie des variables vides
        else:
            return (None, None, image, mask_closing) if return_image == True else (None, None)
"""

#fichier utilities.py

"""
def tracage_nord_est(drone, image):
    cv2.line(image, 
            (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 
            (int(-(drone.camera.x_imageCenter+50)*sin(drone.vehicle.attitude.yaw)), int(drone.camera.y_imageCenter*cos(drone.vehicle.attitude.yaw))), 
            (0, 0, 0), 2)
    cv2.line(image, 
            (drone.camera.x_imageCenter, drone.camera.y_imageCenter), 
            (int(drone.camera.x_imageCenter*(-cos(drone.vehicle.attitude.yaw))), int(-(drone.camera.y_imageCenter-50)*sin(drone.vehicle.attitude.yaw))), 
            (0, 0, 255), 2)
"""