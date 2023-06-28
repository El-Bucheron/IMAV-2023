# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
import cv2
import numpy as np
from commande_drone import Drone
from time import sleep

drone = Drone()

try:
    while True:
        
        altitude = drone.vehicle.rangefinder.distance 
        #--- Capturer le videocamera 
        drone.camera.camera.capture(drone.camera.rawCapture, format="bgr")
        frame = drone.camera.rawCapture.array
        drone.camera.rawCapture.truncate(0)
        #------------- Image processing for white squares -------------
        blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
        hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
        # Définition des limites maximales et minimales de filtre pour garder la couleur blanche en HLS
        lower_bound = (0,150,0)
        upper_bound = (255,255,255)
        mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
        # Closing detected elements
        closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
        mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
        contours, _ = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #--------------White square corners ---------------------------
        for c in contours:
            # pour identifier un carre
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            area = cv2.contourArea(c)
            
            #if 10000*altitude**-2 < area < 60000*altitude**-2 and len(approx) == 4:
            (_, _, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            if ar >= 0.90 and ar <= 1.10:  # Square filter
                x_centerPixel_target = int(np.mean(c, axis=0)[0][0])
                y_centerPixel_target = int(np.mean(c, axis=0)[0][1])
                cv2.line(frame, (x_centerPixel_target, y_centerPixel_target-10), (x_centerPixel_target, y_centerPixel_target+10), (0, 0, 255), 2)
                cv2.line(frame, (x_centerPixel_target-10, y_centerPixel_target), (x_centerPixel_target+10, y_centerPixel_target), (0, 0, 255), 2)
        cv2.imshow("Masque", mask_closing)
        cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        cv2.imshow("Image", frame)
        cv2.waitKey(0)

finally:
    # Fermeture des fenêtres
    cv2.destroyAllWindows()
