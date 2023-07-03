# Imports
import cv2
import numpy as np
from picamera import PiCamera

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

vertical_res = int(1.5*480)
horizotal_res = int(1.5*640)
picamera = PiCamera()
picamera.resolution = (horizotal_res, vertical_res)
photo = np.empty((vertical_res * horizotal_res * 3), dtype=np.uint8)

try:
    for frame in picamera.capture_continuous(photo, format="bgr", use_video_port=True):
        
        photo = frame.reshape((vertical_res, horizotal_res, 3))
        gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)
        
        # Récupération des coins de l'échiquier 
        ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
        
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
        
        if cv2.waitKey(1) &0xFF == ord('q'):
            print("Fin de la prise de photos")
            break


    


    # Calcul et affichage des matrices de caméra et de distortion
    print("Calcul des matrices en cours")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Matrice de la caméra obtenue :")
    print(mtx)
    print("Matrice de distortion obtenue :")
    print(dist)

    newcameramtx, zoi = cv2.getOptimalNewCameraMatrix(mtx, dist, picamera.resolution, 1, picamera.resolution) 
    image_corrige = cv2.undistort(photo, mtx, dist, None, newcameramtx)
    image_corrige = image_corrige[zoi[1]:zoi[1]+zoi[3], zoi[0]:zoi[0]+zoi[2]]
    cv2.imshow('Image corrige', image_corrige)


    #Initialisation de l'erreur totale
    erreur_totale = 0
    # On calcule l'erreur pour chaque image utilisée pour la calibration
    for i in range(len(objpoints)):
        # On détermine l'image projetée
        iamgeProjetee, _ =cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        # On calcule la norme entre l'image projetée et l'image obtenue et on ajoute l'erreur à l'erreur totale
        erreur_totale += cv2.norm(imgpoints[i], iamgeProjetee, cv2.NORM_L2)/len(iamgeProjetee)
    # Calcul de l'erreur moyenne
    erreur_moyenne = [erreur_totale/len(objpoints)]
    print("Erreur moyenne = " + str(erreur_moyenne[0]))
    # Affichage de l'erreur moyenne pour la configuration en usage
    try:
        print("Erreur de la configuration ecritre : " + np.loadtxt("erreurMoyenne" + str(horizotal_res) + "x" + str(vertical_res) + ".txt", delimiter=',')[0])
    except:
        pass

    # Si la touche pressé est 's' on écrit les matrices dans un fichier .txt, un pour chaque matrice
    print("Appuyer sur 's' pour enregistrer les valeurs")
    if cv2.waitKey(0) & 0xFF == ord("s"):
        print("Matrices enregistrées")
        np.savetxt("cameraMatrix" + str(horizotal_res) + "x" + str(vertical_res) + ".txt", mtx, delimiter=',')
        np.savetxt("cameraDistortion" + str(horizotal_res) + "x" + str(vertical_res) + ".txt", dist, delimiter=',')
        np.savetxt("erreurMoyenne" + str(horizotal_res) + "x" + str(vertical_res) + ".txt", erreur_moyenne, delimiter=',')
    else:
        print("Matrices défaussées")


finally:
    cv2.destroyAllWindows()

