#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022
@author: Thomas Pavot
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
from dronekit import LocationGlobalRelative
from picamera import PiCamera
from picamera.array import PiRGBArray
from utilities import *
import imutils
from imutils.video import FPS

class Detection:
    def __init__(self):

        #--------------- Resolution ---------------------------
        # Focal length and sensors dimensions for Pi camera
        # See: https://www.raspberrypi.com/documentation/accessories/camera.html 
        focal_length = 3.60   # Focal length [mm]
        self.horizotal_res = 640   # Horizontal resolution (x dimension) [px] 
        self.vertical_res = 480    # Vertical resolution (y dimension) [px]
        sensor_length = 3.76  # Sensor length (x dimension) [mm]
        sensor_height = 2.74  # Sensor length (y dimension) [mm]  
        self.dist_coeff_x = sensor_length/(focal_length*self.horizotal_res)
        self.dist_coeff_y = sensor_height/(focal_length*self.vertical_res)
        self.x_imageCenter = int(self.horizotal_res/2)
        self.y_imageCenter = int(self.vertical_res/2)  

        # Intialisation de la picamera
        self.camera = PiCamera()
        self.camera.resolution = (self.horizotal_res, self.vertical_res)
        self.camera.framerate = 30
        self.rawCapture = PiRGBArray(self.camera, size=(self.horizotal_res, self.vertical_res))

        # Récupération du chemin d'accès global
        self.package_path = os.getcwd()
        while self.package_path[-9:] != "IMAV_2023":
            self.package_path = os.path.dirname(self.package_path)
        sys.path.insert(0, self.package_path)

        # Camera calibration path
        calib_camera_path = self.package_path + "/config/camera/"
        self.camera_matrix = np.loadtxt(calib_camera_path+'cameraMatrix.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(calib_camera_path+'cameraDistortion.txt', delimiter=',')
        self.matrice_camera_corrigee, self.ROI_camera_corrigee = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.camera_distortion, self.camera.resolution, 1, self.camera.resolution)

        # Definir le dictionnaire aruco 
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters  = aruco.DetectorParameters_create()




    # Fonction permettant de prendre une photo avec la camera
    # On peut également choisir de stocker la photo si on lui fournit en argument un chemin pour stocker la photo
    def prise_photo(self, chemin_photo=""):
        photo = np.empty((self.vertical_res * self.horizotal_res * 3), dtype=np.uint8)
        self.camera.capture(photo, 'bgr')
        photo = photo.reshape((self.vertical_res, self.horizotal_res, 3))
        photo_corrigee = cv2.undistort(photo, self.camera_matrix, self.camera_distortion, None, self.matrice_camera_corrigee)
        photo_corrigee = photo_corrigee[self.ROI_camera_corrigee[1]:self.ROI_camera_corrigee[1]+self.ROI_camera_corrigee[3],
                                        self.ROI_camera_corrigee[0]:self.ROI_camera_corrigee[0]+self.ROI_camera_corrigee[2]]
        if chemin_photo != "":
            cv2.imwrite(chemin_photo, photo_corrigee)
        else:
            return photo_corrigee


    def Detection_position(self):

        # Délai pour que la caméra se stabilise
        time.sleep(2)

        # Seuils de taille pour catégoriser les formes
        petite_seuil_min = 100
        petite_seuil_max = 3000
        moyenne_seuil_min = 3000
        moyenne_seuil_max = 5000
        taille_min_forme = 700  # Seuil pour exclure les formes trop petites

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Lecture de l'image depuis le flux
            image = frame.array           
            # Redimensionnement de l'image
            image = imutils.resize(image, width=600)           
            # Conversion de l'image de l'espace de couleurs BGR (Bleu-Vert-Rouge) à l'espace de couleurs HSV (Teinte-Saturation-Value)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        
            # On définit la gamme de couleur de bleu que l'on souhaite
            lower_blue = np.array([105, 105, 25])
            upper_blue = np.array([160, 255, 200])  
            # Création d'un masque binaire à partir de l'image HSV pour les zones bleues
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)            
            # Création d'un masque binaire inverse pour le reste de l'image
            mask_white = cv2.bitwise_not(mask_blue)            
            # Application du masque binaire bleu à l'image RGB pour conserver les zones bleues
            seg_img_blue = cv2.bitwise_and(image, image, mask=mask_blue)            
            # Création d'une image blanche de la même taille que l'image d'origine
            white_img = np.ones_like(image, dtype=np.uint8) * 255            
            # Application du masque binaire inverse à l'image blanche pour avoir le reste en blanc
            seg_img_white = cv2.bitwise_and(white_img, white_img, mask=mask_white)            
            # Combinaison des deux images pour obtenir le résultat final
            result = cv2.bitwise_or(seg_img_blue, seg_img_white)            
            # Recherche des contours des objets et affichage
            contours, hier = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = cv2.contourArea(contour)

                # Vérifier si l'aire est supérieure au seuil de taille minimum
                if area > taille_min_forme:
                    # Catégoriser l'aire en debout, assis et allongé en fonction des seuils de taille
                    if petite_seuil_min < area < petite_seuil_max:
                        category = 'debout'
                    elif moyenne_seuil_min < area < moyenne_seuil_max:
                        category = 'assis'
                    else:
                        category = 'allongé'

                    # Dessiner le rectangle autour de l'objet bleu
                    cv2.drawContours(result, [contour], 0, (0, 0, 255), 3)
                    # Écrire la catégorie au centre de l'objet bleu
                    cv2.putText(result, category, (x + int(w / 2), y + int(h / 2)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Affichage de l'image résultante
            cv2.imshow('Video', result)

            # Sortir de la boucle si la touche 'q' est pressée
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            # Effacement du tampon de capture pour la prochaine image
            self.rawCapture.truncate(0)

        # Fermeture des fenêtres d'affichage
        cv2.destroyAllWindows()

          
    
    def Detection_carre_bleu(self):
      
        # Délai pour que la caméra se stabilise
        time.sleep(2)

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # Lecture de l'image depuis le flux
            image = frame.array
            # Redimensionnement de l'image
            image = imutils.resize(image, width=600)
            cv2.imshow('image_originale', image)
            # Conversion de l'image de l'espace de couleurs BGR (Bleu-Vert-Rouge) à l'espace de couleurs HSV (Teinte-Saturation-Value)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # Définition de la gamme de couleur bleue souhaitée
            lower_blue = np.array([95, 105, 25])
            upper_blue = np.array([180, 255, 200])
            # Création d'un masque binaire à partir de l'image HSV pour les zones bleues
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            # Création d'un masque binaire inverse pour le reste de l'image
            mask_white = cv2.bitwise_not(mask_blue)
            # Application du masque binaire bleu à l'image RGB pour conserver les zones bleues
            seg_img_blue = cv2.bitwise_and(image, image, mask=mask_blue)
            # Création d'une image blanche de la même taille que l'image d'origine
            white_img = np.ones_like(image, dtype=np.uint8) * 255
            # Application du masque binaire inverse à l'image blanche pour avoir le reste en blanc
            seg_img_white = cv2.bitwise_and(white_img, white_img, mask=mask_white)
            # Combinaison des deux images pour obtenir le résultat final
            result = cv2.bitwise_or(seg_img_blue, seg_img_white)
            # Recherche des contours des objets et affichage
            contours, _ = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            i = 0
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 70 and h > 70:
                    if i == 0:
                        i = 1
                        continue
                    approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
                    cv2.drawContours(result, [contour], 0, (0, 0, 255), 3)
                    M = cv2.moments(contour)
                    if M['m00'] != 0.0:
                        x = int(M['m10']/M['m00'])
                        y = int(M['m01']/M['m00'])

                    if len(approx) < 5:
                        cv2.putText(result, 'Carre', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    else:
                        cv2.putText(result, 'Autre', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Affichage de l'image après avoir dessiné les contours
            cv2.imshow('shapes', result)

            # Sortir de la boucle si la touche 'q' est pressée
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            # Effacement du tampon de capture pour la prochaine image
            self.rawCapture.truncate(0)

        # Fermeture des fenêtres d'affichage
        cv2.destroyAllWindows()

          

    # Fonction servant trouver les arucos vus par la caméra et renvoyant les coordonnées de son centre
    # Si aucun n'aruco n'est détecté, on renvoie des variables vides      
    def detection_aruco(self):
        # Capture et traitement de l'image prise par la picaméra
        self.camera.capture(self.rawCapture, format="bgr")
        frame = self.rawCapture.array
        self.rawCapture.truncate(0)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Recherche d'aruco dans l'image prise
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                  cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
        # Si la longueur du tuple "corners", n'est pas de vide, i.e. si au moins un aruco est détecté
            
        if len(corners) != 0 :            
            # On calcule la moyenne en x et y des arrêtes de l'aruco
            x_centerPixel_target = int((corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0])*.25)
            y_centerPixel_target = int((corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1])*.25)
            aruco_id = ids.flatten()[0]  # Select the first ArUco id from the list
            # On renoive les coordronnées calculées
            return x_centerPixel_target, y_centerPixel_target, aruco_id
        # Si l'aruco n'a pas été détecté, on renvoie des variables vides
        else:
            return None, None, None


    # Fonction servant à détecter l'aruco quand l'altitude est trop élevée pour qu'on le détecte directement avec la fonction "detection_aruco"
    # Pour cela, on assume que l'aruco vu du ciel, après quelques corrections d'images, est un carré blanc
    # La fonction renvoie les coordonnées du carré blanc si elle en trouve un ou des variables vides sinon
    def detection_carre_blanc(self, altitude):
        # Capture et traitement de l'image prise par la picaméra
        self.camera.capture(self.rawCapture, format="bgr")
        frame = self.rawCapture.array
        self.rawCapture.truncate(0)
        #------------- Image processing for white squares -------------
        blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
        # Définition des limites maximales et minimales de filtre pour garder la couleur blanche en HLS
        lower_bound = (0,150,0)
        upper_bound = (255,255,255)
        # Création du masque
        mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
        # Matrice de strucutre
        closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        # masque permettant de remplir le centre d'un contour avec une unique couleur
        mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
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
            # Si le contour possède 4 côtés, et est 
            if 10000*altitude**-2 < area < 60000*altitude**-2 and len(approx) == 4:
                # Récupération de la longueur et largeur du contour
                (_, _, w, h) = cv2.boundingRect(approx)
                # Si le rapport longueur/largeur est compris entre 0.9 et 1.1, i.e. qu'il s'agit d'un carré
                if 0.9 <= (w / float(h)) <= 1.10:
                    # Calcule du centre du carré avec la moyenne des coordonnées en X et Y
                    x_centerPixel_target = int(np.mean(c, axis=0)[0][0])
                    y_centerPixel_target = int(np.mean(c, axis=0)[0][1])
                    # Si le pixel central est blanc, i.e. que le carré est blanc
                    if mask_closing[y_centerPixel_target,x_centerPixel_target] == 255:
                        # On renvoie les coordonnées du centre du carré
                        return x_centerPixel_target, y_centerPixel_target
        # Si aucun carré blanc n'a été détecté, on renvoie des variables vides
        return None, None
