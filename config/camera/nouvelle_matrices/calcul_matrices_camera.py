# Imports
import cv2
import numpy as np
import os.path
from picamera import PiCamera
from time import sleep

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 25, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Initialisation du compteur d'images prise en compte pour la calibration
images_valides=0

vertical_res = 480
horizotal_res = 640
picamera = PiCamera()
picamera.resolution = (horizotal_res, vertical_res)
photo = np.empty((vertical_res * horizotal_res * 3), dtype=np.uint8)

path_camera_matrix = "cameraMatrix" + str(horizotal_res) + "x" + str(vertical_res) + ".txt"
path_camera_distrotion = "cameraDistortion" + str(horizotal_res) + "x" + str(vertical_res) + ".txt"
path_image_controle = "ImageControle" + str(horizotal_res) + "x" + str(vertical_res) + ".jpg"

picamera.start_preview()
sleep(15)
picamera.stop_preview()

try:
    picamera.start_preview()
    for frame in picamera.capture_continuous(photo, format="bgr", use_video_port=True):
        
        # Prise de la photo
        photo = frame.reshape((vertical_res, horizotal_res, 3))
        gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)        
        # Récupération des coins de l'échiquier 
        ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
        # Attenteq
        if cv2.waitKey(1) &0xFF == ord('q'):
            print("Fin de la prise de photos")
            break
        # Si les coins n'ont pas été détectés, on indique que l'image n'est pas valide et on interrompt l'itération en cours 
        if ret == False:
            print("Echiquier non détecté, image non prise en compte")
            continue
        
        # Si les coins ont été détectés on les trace et les affiche pour l'utilisateur
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        cv2.drawChessboardCorners(photo, (7,7), corners2, ret)
        cv2.imshow('img', photo)
        
        #Si l'image a été validée on ajoute l'image aux tableaux de données et on incrémente le compteur d'image
        print("Photo valide prise. Nombre total de photos prises : " + str(images_valides))
        objpoints.append(objp)
        imgpoints.append(corners2)
        images_valides+=1
        


    # Calcul et affichage des matrices de caméra et de distortion
    cv2.destroyAllWindows()
    print("Calcul des matrices en cours")
    _, new_mtx, new_dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Matrice de la caméra obtenue :")
    print(new_mtx)
    print("Matrice de distortion obtenue :")
    print(new_dist)
    
    if not os.path.isfile(path_image_controle):
        print("Faire en sorte que l'échiquier prenne tout l'espace sur la caméra et appuyez sur 's'")
        picamera.start_preview()
        while cv2.waitKey(0) & 0xFF != ord("s"):
            pass
        picamera.capture(path_image_controle)
        picamera.stop_preview()
    imageControle = cv2.imread(path_image_controle)

        

    # Correction d'image avec les nouvelles matrices
    new_cameramtx, new_zoi = cv2.getOptimalNewCameraMatrix(new_mtx, new_dist, picamera.resolution, 1, picamera.resolution)
    nouvelle_correction = cv2.undistort(imageControle, new_mtx, new_dist, None, new_cameramtx)
    nouvelle_correction = nouvelle_correction[new_zoi[1]:new_zoi[1]+new_zoi[3], new_zoi[0]:new_zoi[0]+new_zoi[2]]
    cv2.imshow('Image de contrôle : nouvelles matrices', nouvelle_correction)
    
    # Correction d'image avec les anciennnes matrices (dans le cas où elles existent)
    try:
        old_mtx = np.loadtxt(path_camera_matrix, delimiter=',')
        old_dist = np.loadtxt(path_camera_distrotion, delimiter=',')
        old_cameramtx, old_zoi = cv2.getOptimalNewCameraMatrix(old_mtx, old_dist, picamera.resolution, 1, picamera.resolution)
        ancienne_correction = cv2.undistort(imageControle, old_mtx, old_dist, None, old_cameramtx)
        ancienne_correction = ancienne_correction[old_zoi[1]:old_zoi[1]+old_zoi[3], old_zoi[0]:old_zoi[0]+old_zoi[2]]
        cv2.imshow('Image de contrôle : anciennes matrices', ancienne_correction)
    except FileNotFoundError:
        pass
    

    # Si la touche pressé est 's' on écrit les matrices dans un fichier .txt, un pour chaque matrice
    print("Appuyer sur 's' pour enregistrer les valeurs")
    if cv2.waitKey(0) & 0xFF == ord("s"):
        print("Matrices enregistrées")
        np.savetxt(path_camera_matrix, new_mtx, delimiter=',')
        np.savetxt(path_camera_distrotion, new_dist, delimiter=',')
    else:
        print("Matrices défaussées")


finally:
    cv2.destroyAllWindows()
    picamera.stop_preview()

