"""
Code permettant de calculer les matrices de correction d'images de la caméra

Fonctionnement du code:
    On prend des photos en continu avec la picamera. Sur ces photos, on essaie de trouver un échiquier
    Si un échiqier est trouvé, on garde l'image. On continue ainsi jusqu'à ce que l'utilisateur appuie sur "q"
    On calcule les matrices de caméra et distortion avec les images d'échiqiers sauvergardées.
    On corrige une image de contrôle avec les matrices calculées et les matrices actuelles (si elles ont déjà été calculées)
    On affiche les deux images corrigées et l'utilisateur peut choisir de garder les nouvelles matrices ou de les défaussées
    
Spécificités du codes:
    Les paramètres du codes sont la résolution de la caméra et la longueur des arrêtes des cases de l'échiquier, les matrices changent si on modifie ces paramètres
    Les matrices de correction d'images et l'image de contrôle sont stockés dans un dossier "horizotal_resxvertical_res" -> exemple "640x480"
    Si ce dossier, l'une des matrices de correction ou l'image de contrôle n'existe pas, on le/la crée dans le code
    L'image de calibration peut être n'importe quelle image mais il est plus judicieux de prendre une photo de l'échiquier en gros plan pour observer les déformations
    Pour augmenter la précision de la correction, il faut que le plan de l'échiquier soit parallèle au plan de la lentille
    Il faut au moins une trentaine d'image pour avoir une correction efficace
"""

# Imports
import cv2
import numpy as np
import os.path
from picamera import PiCamera
from time import sleep

# Variables de travail
horizotal_res = int(640*2) # en pixel
vertical_res = int(480*2) # en pixel
taille_case = 25 # en mm

# S'il n'existe pas déjà, on crée le dossier de paramètres de caméra pour la résolution choisie
try:
    os.mkdir(str(horizotal_res) + "x" + str(vertical_res))
except FileExistsError:
    pass

# Chemin vers les (futures) matrices de calibration 
path_camera_matrix = str(horizotal_res) + "x" + str(vertical_res) + "/" +"cameraMatrix.txt"
path_camera_distrotion = str(horizotal_res) + "x" + str(vertical_res) + "/" + "cameraDistortion.txt"
path_image_controle = str(horizotal_res) + "x" + str(vertical_res) + "/" + "imageControle.jpg"

# Configuration de la picamera
picamera = PiCamera()
picamera.resolution = (horizotal_res, vertical_res)
photo = np.empty((vertical_res * horizotal_res * 3), dtype=np.uint8)

# Variable pour stocker les images 
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, taille_case, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Initialisation du compteur d'images prise en compte pour la calibration
images_valides=0


try:
    # On capture en permancence les photos de la picamera 
    for frame in picamera.capture_continuous(photo, format="bgr", use_video_port=True):
        
        # On attend une potentielle frappe de l'utilisateur et si cette touche est "q", on arrête la prise de photo
        if cv2.waitKey(1) &0xFF == ord('q'):
            print("Fin de la prise de photos")
            break
        
        # Redimensionnement du vecteur représentant l'image
        photo = frame.reshape((vertical_res, horizotal_res, 3))
        # Conversion de l'image en couleur en image de nuances de gris
        gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)
        # Récupération des coins de l'échiquier
        ret, corners = cv2.findChessboardCorners(gray, (7,7), None)      
        # Si les coins n'ont pas été détectés, on indique que l'image n'est pas valide et on interrompt l'itération en cours 
        if ret == False:
            print("Echiquier non détecté, image non prise en compte")
            continue
        
        # Si les coins ont été détectés on les trace et on les affiche pour l'utilisateur
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        cv2.drawChessboardCorners(photo, (7,7), corners2, ret)
        cv2.imshow("Prise de photo", photo)
        
        #Si l'image a été validée on ajoute l'image aux tableaux de données et on incrémente le compteur d'image
        print("Photo valide prise. Nombre total de photos prises : " + str(images_valides))
        objpoints.append(objp)
        imgpoints.append(corners2)
        images_valides+=1
        
        # Temporisation pour ne pas avoir trop d'image et donc un temps de calcul trop long 
        sleep(0.2)


    # Calcul et affichage des matrices de caméra et de distortion
    cv2.destroyAllWindows()
    print("Calcul des matrices en cours")
    _, new_mtx, new_dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Matrice de la caméra obtenue :")
    print(new_mtx)
    print("Matrice de distortion obtenue :")
    print(new_dist)
    
    
    # Si l'image de contrôle n'existe pas, on demande à l'utilisateur de la prendre en appuyant sur le bouton ss
    if not os.path.isfile(path_image_controle):
        print("Faire en sorte que l'échiquier prenne tout l'espace sur la caméra et appuyez sur 's'")
        # On capture en permancence les photos de la picamera 
        for frame in picamera.capture_continuous(photo, format="bgr", use_video_port=True):
            # On attend une potentielle frappe de l'utilisateur et si cette touche est "q", on arrête la prise de photo
            if cv2.waitKey(1) &0xFF == ord('s'):
                break
            # Redimensionnement du vecteur représentant l'image
            photo = frame.reshape((vertical_res, horizotal_res, 3))
            # Affichage de l'ilage
            cv2.imshow("", photo)
        # On enregistre la dernière photo prise 
        cv2.imwrite(path_image_controle, photo)
        print("Image ce contrôle prise")
    # On récupère l'iamge de contrôle
    imageControle = cv2.imread(path_image_controle)

        

    # Correction d'image avec les nouvelles matrices
    # Calcul de la nouvelle matrice de correction et la zone d'intérêt liée à cette matrice
    new_cameramtx, new_zoi = cv2.getOptimalNewCameraMatrix(new_mtx, new_dist, picamera.resolution, 1, picamera.resolution)
    # Correction de l'image
    nouvelle_correction = cv2.undistort(imageControle, new_mtx, new_dist, None, new_cameramtx)
    # Rognage de l'image pour faire correspondre l'image à la correction
    nouvelle_correction = nouvelle_correction[new_zoi[1]:new_zoi[1]+new_zoi[3], new_zoi[0]:new_zoi[0]+new_zoi[2]]
    # Affichage de l'image corrigée
    cv2.imshow('Image de contrôle : nouvelles matrices', nouvelle_correction)
    
    
    # Correction d'image avec les anciennnes matrices (dans le cas où elles existent)
    try:
        # Récupération des anciennes matrices
        old_mtx = np.loadtxt(path_camera_matrix, delimiter=',')
        old_dist = np.loadtxt(path_camera_distrotion, delimiter=',')
        # Calcul de l'ancienne matrice de correction
        old_cameramtx, old_zoi = cv2.getOptimalNewCameraMatrix(old_mtx, old_dist, picamera.resolution, 1, picamera.resolution)
        # Correction de l'image
        ancienne_correction = cv2.undistort(imageControle, old_mtx, old_dist, None, old_cameramtx)
        # Rognage de l'image pour faire correspondre l'image à la correction
        ancienne_correction = ancienne_correction[old_zoi[1]:old_zoi[1]+old_zoi[3], old_zoi[0]:old_zoi[0]+old_zoi[2]]
        # Affichage de l'image corrigée
        cv2.imshow('Image de contrôle : anciennes matrices', ancienne_correction)
    except FileNotFoundError:
        pass
    

    # On attend la prochaine touche de l'utilisateur et si cette touche est 's' on sauvegarde les matrices
    print("Appuyer sur 's' pour enregistrer les valeurs")
    if cv2.waitKey(0) & 0xFF == ord("s"):
        print("Matrices enregistrées")
        np.savetxt(path_camera_matrix, new_mtx, delimiter=',')
        np.savetxt(path_camera_distrotion, new_dist, delimiter=',')
    else:
        print("Matrices défaussées")

# Suppression des fenêtres opencv
finally:
    cv2.destroyAllWindows()

