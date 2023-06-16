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
from math import sqrt
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray
from utilities import get_distance_metres, get_GPS_location, get_distance_angle_picture
import imutils
from imutils.video import FPS

class Detection:
  def __init__(self, id_to_find=57):
      
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

    #--------------- Boolean variables --------------------
    self.aruco_seen = False
    self.good_aruco_found = False
    self.white_square_seen = False

    #--------------- Counter variables --------------------
    # self.found_count = 0
    # self.not_found_count = 0
    
    #--------------- Camera parametres --------------------
    self.camera = PiCamera()
    self.camera.resolution = (self.horizotal_res, self.vertical_res)
    self.camera.framerate = 30
    self.rawCapture = PiRGBArray(self.camera, size=(self.horizotal_res, self.vertical_res))
  
    #--------------- ArUco parametres --------------------
    self.id_to_find = id_to_find
    self.marker_size = 5 #- [cm]

  
    """
    self.img_compteur = 0
    dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
    parent_dir = '/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images'
    self.path = os.path.join(parent_dir, dossier)
    os.mkdir(self.path)
    """
    # self.f = open(str(self.path) + "log.txt", "a")
  
    #--- Camera calibration path
    calib_path  = "/home/housso97/Desktop/2023/random IMAV 2023/Test vol drone 14 juin/matrices_camera/"
    self.camera_matrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    self.camera_distortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
  
    #--- Definir le dictionnaire aruco 
    self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    self.parameters  = aruco.DetectorParameters_create()
  
    self.closeToAruco = False
    
    # Initialsiation du tracker
    self.tracker = cv2.TrackerCSRT_create()

    #--------------- Saved Markers ---------------------------
    # self.saved_markers = {-1: (LocationGlobalRelative(48.58111,7.764722,0), False)}
    
  #Fonction permettant de prendre une photo avec la camera
  def prise_photo(self, chemin_photo):
    photo = np.empty((self.vertical_res * self.horizotal_res * 3), dtype=np.uint8)
    self.camera.capture(photo, 'bgr')
    photo = photo.reshape((self.vertical_res, self.horizotal_res, 3))
    matrice_camera_corrigee, ROI = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.camera_distortion, self.camera.resolution, 1, self.camera.resolution)
    photo_corrigee = cv2.undistort(photo, self.camera_matrix, self.camera_distortion, None, matrice_camera_corrigee)
    photo_corrigee = photo_corrigee[ROI[1]:ROI[1]+ROI[3], ROI[0]:ROI[0]+ROI[2]]
    cv2.imwrite(chemin_photo, photo_corrigee)
    
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

        # Définition de la gamme de couleur bleue souhaitée
       # lower_blue = np.array([105, 105, 25])
        #upper_blue = np.array([150, 255, 255])
        
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
        contours, hier = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
          
          
    
  def Detection_aruco4(self): # sans tracking

    # Délai pour que la caméra se stabilise
    time.sleep(2)

    for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
        # Lecture de l'image depuis le flux
        img = frame.array

        # Redimensionnement de l'image
        img = imutils.resize(img, width=600)


        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convertit l'image en nuance de gris
    # On récupère les polygones détectés, les id et les code qui ne sont pas dans le dict
        boxs, ids, rejected = aruco.detectMarkers(imgGray, self.aruco_dict, parameters=self.parameters)

        
        aruco.drawDetectedMarkers(img, boxs, borderColor=(0,255,0)) #affiche les aruco détectés et valides sur l'image
        if not ids is None:
            for box, di in zip(boxs, ids):
                # print(di)
                cv2.putText(img, str(di), (int(box[0][0][0]), int(box[0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                    (255, 0, 255), 1)
                            
        cv2.imshow("Image", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        # Effacement du tampon de capture pour la prochaine image
        self.rawCapture.truncate(0)


    cv2.destroyAllWindows()
          
          
      

  def Detection_aruco3(self):  # amelioration Maya
          
      # Délai pour que la caméra se stabilise
    time.sleep(2)


    # Variables de suivi
    tracking_initialized = False
    bbox = None # La variable bbox est vide

    for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
        # Lecture de l'image depuis le flux
        img = frame.array

        # Redimensionnement de l'image
        img = imutils.resize(img, width=600)

        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convertit l'image en nuance de gris
        
        # On récupère les polygones détectés, les id et les code qui ne sont pas dans le dict
        boxs, ids, rejected = aruco.detectMarkers(imgGray, self.aruco_dict, parameters=self.parameters)

        aruco.drawDetectedMarkers(img, boxs, borderColor=(0, 255, 0))  # Affiche les aruco détectés et valides sur l'image
        if not ids is None:
            for box, di in zip(boxs, ids):
                cv2.putText(img, str(di), (int(box[0][0][0]), int(box[0][0][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                            (255, 0, 255), 1)

                # Récupère les coins du carré vert entourant l'ArUco
                corners = np.float32(box[0]).reshape(-1, 2)

                if not tracking_initialized:
                    # Initialise le tracker CSRT
                    bbox = (np.min(corners[:,0]),np.min(corners[:,1]),np.max(corners[:,0])-np.min(corners[:,0]),np.max(corners[:,1])-np.min(corners[:,1]))
                    self.tracker.init(img, bbox)
                    tracking_initialized = True
                else:
                    # Effectue le suivi avec le tracker CSRT
                    success, bbox = self.tracker.update(img)

                    if success:
                        # Dessine le rectangle de suivi
                        x, y, w, h = [int(v) for v in bbox]
                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow("Image", img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        # Effacement du tampon de capture pour la prochaine image
        self.rawCapture.truncate(0)


    cv2.destroyAllWindows()
        

  def Detection_aruco2(self):   # amelioration Matthieu
      
    # Délai pour que la caméra se stabilise
    time.sleep(2)
     
    # Ci dessous se trouve trois types de tracker différents 
    #tracker = cv2.TrackerKCF_create()
    #tracker = cv2.TrackerCSRT_create()
    #tracker = cv2.TrackerMIL_create()

    arucoRepere = None    # La variable aruco est vide
    fps = None      # La variable fps est vide (nb d'images par secondes)


    for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):  # Boucle de capture d'images en continu à partir de la caméra
        
        # Lecture de l'image depuis le flux
        image = frame.array

        # Redimensionnement de l'image
        image = imutils.resize(image, width=600)

        #L'image est convertie en nuances de gris
        frameGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        if arucoRepere is None: # Si la variable aruco est vide
            arucoCorner,_,_ = aruco.detectMarkers(frameGray, self.aruco_dict, parameters=self.parameters) # On cherche un code aruco
            if len(arucoCorner) != 0: # Si la longueur des côtés de l'aruco sont différents de zéro
                corners = arucoCorner[0].reshape((4,2)) # on récupère les coins de l'aruco
                print(corners) # on affiche les coordonnées des coins
                min_x = min(corners[0][0], corners[1][0], corners[2][0], corners[3][0]) # on récupère la coordonnées du coin ayant la plus petite distance en abscisse
                max_x = max(corners[0][0], corners[1][0], corners[2][0], corners[3][0]) # on récupère la coordonnées du coin ayant la plus grande distance en abscisse
                min_y = min(corners[0][1], corners[1][1], corners[2][1], corners[3][1])# on récupère la coordonnées du coin ayant la plus petite distance en ordonnée
                max_y = max(corners[0][1], corners[1][1], corners[2][1], corners[3][1])# on récupère la coordonnées du coin ayant la plus grande distance en ordonnée
                
                #on réalise le tracking du coin ayant la plus petite distance en abscisse et en ordonnée ainsi que ses côtés respectifs
                self.tracker.init(image, [int(min_x),
                                    int(min_y),
                                    int(max_x-min_x),
                                    int(max_y-min_y)])
                
                fps = FPS().start() # On démarre le compte des images par seconde
                arucoRepere = 1     # La variable aruco prend la valeur de 1
        
        # Si un aruco a été détecté :
        if arucoRepere is not None:
            (success, box) = self.tracker.update(image) # On effectue le suivi avec le tracker
            if success:
                # On dessine le rectangle de suivi
                (x, y, w, h) = [int(v) for v in box] 
                cv2.rectangle(image, (x, y), (x + w, y + h),(0, 255, 0), 2)
                
            fps.update() # mise a jour du compteur fps
            fps.stop() # arrêt du compteur de fps
            
            # Affichage du fps moyen
            info = [
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())), ]
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(image, text, (10, 480 - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
        cv2.imshow("Frame", image) # On affiche l'image avec les modifications apportées
        
        # En appuyant sur la touche q, on sort de la boucle
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        
        # Effacement du tampon de capture pour la prochaine image
        self.rawCapture.truncate(0)

    # Fermeture de toutes les fenêtres
    cv2.destroyAllWindows()
          
      
      
  

  def Detection_aruco(self, latitude, longitude, altitude, heading, saved_markers, id_to_test, research_whiteSquare):
    # Start time to measure image processing delay
    start_time = time.time()
    
    # Boolean variables reset
    self.aruco_seen = False
    self.good_aruco_found = False
    self.white_square_seen = False
    print("Id to test input detection: %s" % id_to_test)
    #print("[visiont] New image. ID to test: %s. Number of saved items: %s." % (id_to_test, len(saved_markers)))
    x_pixel_target_out = None
    y_pixel_target_out = None
    name = "Text"
    new_location_found = False
    
    #--- Capturer le videocamera 
    self.camera.capture(self.rawCapture, format="bgr")
    frame = self.rawCapture.array
    self.rawCapture.truncate(0)
    
    #frame = cv2.imread("/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images/2022_09_07-12:17:38_PM/mask_closingTest_1_Img_29_lat_48.5809508lon_7.7648142alt_23.8799991607666.png")
    
    font = cv2.FONT_HERSHEY_PLAIN  # Text font for frame annotation
    
    """print("Captured frame: " + str(type(frame)))  
    frame = cv2.flip (frame,-1)
    print("Flipped frame: " + str(type(frame)))
    cv2.waitKey(33)"""
    
    self.img_compteur+=1
    
    ########################## traitement pour aruco
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    
    #blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
    #hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
    #lower_bound = (0,200,0)     # Select white color in HLS space
    #upper_bound = (255,255,255)
    #mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
  
    #-- Trouver tous les marquers dans l'image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                              cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
    # print(ids.flatten()[0])
    # print(type(ids))
    # print("ids : "+str(ids))

    #--------------- Detection ArUco Tags ---------------------------
    if ids is not None : # and ids[0] == Detection.id_to_find:
      # Boolean update
      self.aruco_seen = True
      # self.found_count += 1
      
      aruco_id = ids.flatten()[0]  # Select the first ArUco id from the list
      x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
      y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
      x_centerPixel_target = int(x_sum*.25)
      y_centerPixel_target = int(y_sum*.25)
      x_pixel_target_out = x_centerPixel_target
      y_pixel_target_out = y_centerPixel_target
      arrete_marker_pxl = sqrt((corners[0][0][0][0]-corners[0][0][1][0])**2+(corners[0][0][0][1]-corners[0][0][1][1])**2)
        
      cv2.line(frame, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
      cv2.line(frame, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)
      cv2.putText(frame, str(aruco_id)+"a", (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
      # Estimating marker location from vision
      distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                 x_centerPixel_target, y_centerPixel_target,
                                                                 altitude, self.dist_coeff_x, self.dist_coeff_y)
      current_location = LocationGlobalRelative(latitude, longitude, 0)
      estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

      # saved_location = saved_markers[id_to_test][0]  # Compare to id_to_test saved location
      # distance_meters = get_distance_metres(estimated_location, saved_location)
      # cv2.putText(frame, str(aruco_id), (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
      #print(distance_meters) 
      # If the white square of interest is located at the ArUco place
      saved_markers[aruco_id] = (estimated_location, True)  # Save Aruco id and its location
        # print(saved_markers[aruco_id])
        # saved_markers[aruco_id][1] = True  # Saved Aruco markers explored, not interesting
        #saved_markers.pop(id_to_test)                      # Remove current white square id

      if aruco_id == self.id_to_find:
        self.good_aruco_found = True
        
    #--------------- Detection White Squares ------------------------
    elif research_whiteSquare == True :      
      #------------- Image processing for white squares -------------
      blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
      hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
      lower_bound = (0,230,0)     # Select white color in HLS space
      upper_bound = (255,255,255)
      mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
      name = "Test_1_Img_" + str(self.img_compteur) + "_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)
      cv2.imwrite(os.path.join(self.path, "hls_"+name +".png"), mask_hls)
      # Closing detected elements
      closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
      mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
      contours, hierarchy = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #print ("aire max : "+str(30000*altitude**-1.743))
      #print ("aire min : "+str(25000*altitude**-1.743))

      #--------------White square corners ---------------------------
      for c in contours:
        # pour identifier un carre
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)
        
        #print ("aire du carre : "+str(area))
        #print( "alt : "+ str(altitude))
        
        #--------------- Altitude and square filters ------------------
        if altitude == 0.0:
          print("[visiont] WARNING! Null altitude null sent by rangefinder.")
          # self.f.write("[visiont] WARNING! Null altitude null sent by rangefinder.")
          break
        if area < 60000*altitude**-2  and area > 10000*altitude**-2 and len(approx) ==4 and altitude > 3:
          # print("Detection area correspondant")
          (x, y, w, h) = cv2.boundingRect(approx)
          ar = w / float(h)
          #print("ar : "+str(ar))
          if ar >= 0.90 and ar <= 1.10:  # Square filter
            # print ("Detection carre blanc OK")
            # cv2.drawContours(frame, [c], -1, (0, 0, 255), 1)
            x_centerPixel_target = np.mean(c, axis=0)[0][0]
            y_centerPixel_target = np.mean(c, axis=0)[0][1]
            arrete_marker_pxl = sqrt(area)
            
            pixelTest = mask_closing[int(y_centerPixel_target),int(x_centerPixel_target)]
            #print("pixelTest : "+str(pixelTest))
            if pixelTest == 255 :  #verifie couleur du carre detecte 255 c est blanc
              # Boolean and counter update
              self.white_square_seen = True
              # self.found_count += 1
              
              # cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
              cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 0, 255), 2)
              cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 0, 255), 2)

              # Estimating marker location from vision
              distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                         x_centerPixel_target, y_centerPixel_target,
                                                                         altitude, self.dist_coeff_x, self.dist_coeff_y)
              current_location = LocationGlobalRelative(latitude, longitude, 0)
              estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

              # White square found and compared to dictionary
              new_location_found = True
              white_square_id = 0
              for id_markers in saved_markers:
                saved_location = saved_markers[id_markers][0]
                distance_meters = get_distance_metres(estimated_location, saved_location)
                print(distance_meters)
                # print("distance_meters entre la position du marker teste et l estimation de la nouvelle image : "+str(distance_meters))

                # White square already checked with location fusion
                if distance_meters < 10:
                  new_location_found = False
                  white_square_id = id_markers
                  # print("Distance less than 3m")
                  # new_location_found = False
                  if white_square_id == id_to_test:
                    # print("Pixel target out sent: %s" % white_square_id)
                    x_pixel_target_out = x_centerPixel_target
                    y_pixel_target_out = y_centerPixel_target

                  # Location already found
                #else:
                #  new_location_found = True
                  # x_pixel_target_out = x_centerPixel_target
                  # y_pixel_target_out = y_centerPixel_target

              # Storing new white squares in dictionary
              if new_location_found:
                if max(saved_markers.keys()) <= 1000:
                  white_square_id = 1001        # First white square with id 1001
                else:
                  max_id = max(saved_markers.keys())
                  white_square_id = max_id + 1  # Others white square with growing ids
                print("[visiont] New location found with id %s." % white_square_id)
                # self.f.write("[visiont] New location found with id %s." % white_square_id)
                saved_markers[white_square_id] = (estimated_location, False)

              if white_square_id > 1000:
                cv2.putText(frame, str(white_square_id), (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
              else:
                self.white_square_seen = False
                cv2.putText(frame, str(white_square_id)+"b", (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)

              #cv2.imwrite(os.path.join(self.path, "mask_hls"+name), mask_hls)
              #cv2.imwrite(os.path.join(self.path, "mask_closing"+name), mask_closing)

    if self.aruco_seen == False and self.white_square_seen == False:
      # self.not_found_count += 1
      x_pixel_target_out = None
      y_pixel_target_out = None
      name = "Test_1_Img_" + str(self.img_compteur) + "_no_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)
      #print ("aruco and white square likely not found")
      # print("notfound_count : "+str(self.notfound_count))
    else:
      name = "Test_1_Img_" + str(self.img_compteur) + "_yes_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)

    cv2.circle(frame, (320, 240), 75, (255,255,255), 1)
    cv2.line(frame, (self.x_imageCenter, self.y_imageCenter-20), (self.x_imageCenter, self.y_imageCenter+20), (255, 0, 0), 2)
    cv2.line(frame, (self.x_imageCenter-20, self.y_imageCenter), (self.x_imageCenter+20, self.y_imageCenter), (255, 0, 0), 2)
    
    # End time to measure image processing delay
    end_time = time.time()
    delay = end_time - start_time
    cv2.imwrite(os.path.join(self.path, name+"delay_"+str(delay)+".png"), frame)
    print("Image saved (%s)!" % self.img_compteur)
    
    return x_pixel_target_out, y_pixel_target_out, self.aruco_seen, self.good_aruco_found, self.white_square_seen, saved_markers
