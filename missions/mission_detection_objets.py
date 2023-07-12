# Code permettant d'importer la classe "Drone" et de réaliser la mission de détection d'objets

#Librairies nécessaires
import os
import sys
import cv2
import numpy as np
import imutils
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

# Imports
from commande_drone import Drone
from time import sleep
from dronekit import connect, VehicleMode, LocationGlobalRelative

#Instanciation de l'objet drone
drone = Drone()
while drone.get_mode() != "STABILIZE":
    print("En attente du mode STABILIZE")
while drone.get_mode() != "AUTO":
    print("En attente du mode AUTO")
drone.set_mode("GUIDED")
# Attente du mode "GUIDED"
while drone.get_mode() != "GUIDED":    
    # On affiche l'altitude de vol
    print("Altitude = " + str(drone.vehicle.rangefinder.distance))
    
    
#Choix de l'altitude de vol : 
altitude = 15

#Choix de la zone de vol : le jour de la compétition, les coordonnées GPS du lieu des mannequins : Lat: 50.909228° Lon: 6.226700°

point = LocationGlobalRelative(48.7065031, 7.7342748, altitude)
    
# Décollage

drone.arm_and_takeoff(altitude)

#Vol vers la zone où se trouve les mannequins
drone.goto(point, 1)

# Prise de photo de la zone 
image = drone.camera.prise_photo()
            
# Création du chemin de la photo
chemin_photo = (package_path + "/"                                          # Chemin du dossier
               datetime.now().strftime("%H:%M:%S") + " " +                   # Heure de prise de la photo  
               str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
               str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
               str('%.2f'%(drone.vehicle.rangefinder.distance)) + ".jpg")    # Encodage de l'altitude
# Sauvegarde de la photo
cv2.imwrite(chemin_photo, image)
# Temporisation
sleep(0.5)
print("Photo prise")

#Analyse de l'image 

taille_min_forme = 0  # Seuil pour exclure les formes trop petites, dans la pratique , on peut placer cette taille à 0

if altitude == 10:
        
    # PARAMETRES A 10M

    # Mannequin bleu
    petite_seuil_min_bleu = 0 
    petite_seuil_max_bleu = 1500
    moyenne_seuil_min_bleu = 1500
    moyenne_seuil_max_bleu = 2000

    # Mannequin rouge

    petite_seuil_min_rouge = 0 
    petite_seuil_max_rouge = 1500
    moyenne_seuil_min_rouge = 1500
    moyenne_seuil_max_rouge = 2000
    
elif altitude == 15:
        
    # PARAMETRES A 15M

    #Mannequin bleu 

    petite_seuil_min_bleu = 0 
    petite_seuil_max_bleu = 600
    moyenne_seuil_min_bleu = 600
    moyenne_seuil_max_bleu = 1000

    #Mannequin rouge

    petite_seuil_min_rouge = 0 
    petite_seuil_max_rouge = 600
    moyenne_seuil_min_rouge = 600
    moyenne_seuil_max_rouge= 1000
    
elif altitude == 20:
        
    #PARAMETRES A 20M

    #Mannequin bleu 

    petite_seuil_min_bleu = 0 
    petite_seuil_max_bleu = 450
    moyenne_seuil_min_bleu = 450
    moyenne_seuil_max_bleu = 600

    #Mannequin rouge

    petite_seuil_min_rouge = 0 
    petite_seuil_max_rouge = 450
    moyenne_seuil_min_rouge = 450
    moyenne_seuil_max_rouge = 600
        
else :
    altitude = 15
    print("attention, l'altitude a été mise par défaut")

# La variable image représente la vidéo, il est redimensionné avec resize
image = imutils.resize(image, width=800)

# Conversion de l'image de l'espace de couleurs BGR (Bleu-Vert-Rouge) à l'espace de couleurs HSV (Teinte-Saturation-Value)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# On définit la gamme de couleur de bleu que l'on souhaite ( H va de 0 à 180 , S et V de 0 à 255)
lower_blue = np.array([105, 105, 25])
upper_blue = np.array([150, 255, 255])

# On définit la gamme de couleur de rouge que l'on souhaite
lower_red = np.array([159, 105, 25])
upper_red = np.array([180, 255, 255])    

# Création d'un masque binaire à partir d'une image HSV pour les zones bleues
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

# Création d'un masque binaire à partir d'une image HSV pour les zones rouges
mask_red = cv2.inRange(hsv, lower_red, upper_red)

# Dilater les contours pour fusionner les taches bleues proches
dilated_mask_blue = cv2.dilate(mask_blue, None, iterations=3)

# Dilater les contours pour fusionner les taches rouges proches
dilated_mask_red = cv2.dilate(mask_red, None, iterations=3)

# Création d'un masque binaire inverse pour le reste de l'image (masque blanc)
mask_white = cv2.bitwise_not(cv2.bitwise_or(dilated_mask_blue, dilated_mask_red))

# On applique le masque binaire bleu à l'image RGB pour conserver les zones bleues
seg_img_blue = cv2.bitwise_and(image, image, mask=dilated_mask_blue)

# On applique le masque binaire bleu à l'image RGB pour conserver les zones bleues
seg_img_red = cv2.bitwise_and(image, image, mask=dilated_mask_red)

# On crée une image blanche de la même taille que l'image d'origine
white_img = np.ones_like(image, dtype=np.uint8) * 255

# On applique le masque binaire inverse à l'image blanche pour avoir le reste en blanc
seg_img_white = cv2.bitwise_and(white_img, white_img, mask=mask_white)

# On combine les images segmentées bleues et rouges en les additionnant
result = cv2.add(seg_img_blue, seg_img_red)
result = cv2.bitwise_or(result, seg_img_white)

# On cherche le contour des objets et on l'affiche
contours_blue, _ = cv2.findContours(dilated_mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Trouver les contours des objets rouges et les afficher
contours_red, _ = cv2.findContours(dilated_mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Compteur de zones rouges
red_zones_count = 0

# Compteur de zones bleues
blue_zones_count = 0

#Nombre de mannequins
nb_mannequins = 0

for contour in contours_blue:
    x, y, w, h = cv2.boundingRect(contour)

    area = cv2.contourArea(contour)

    # Vérifier si l'aire est supérieure au seuil de taille minimum
    if area > taille_min_forme:
        # Catégoriser l'aire en debout, assis et allongé en fonction des seuils de taille
        if petite_seuil_min_bleu < area < petite_seuil_max_bleu:
            category = 'assis'
        elif moyenne_seuil_min_bleu < area < moyenne_seuil_max_bleu:
            category = 'debout'
        else:
            category = 'allonge'

        # Dessiner le rectangle autour de l'objet bleu
        cv2.drawContours(result, [contour], 0, (0, 255, 255), 3)
        # Écrire la catégorie au centre de l'objet bleu
        cv2.putText(result, category, (x + int(w / 2), y + int(h / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Incrémenter le compteur de zones bleues
        blue_zones_count += 1

for contour in contours_red:
    x, y, w, h = cv2.boundingRect(contour)

    area = cv2.contourArea(contour)

    # Vérifier si l'aire est supérieure au seuil de taille minimum
    if area > taille_min_forme:
        # Catégoriser l'aire en debout, assis et allongé en fonction des seuils de taille
        if petite_seuil_min_rouge < area < petite_seuil_max_rouge:
            category = 'assis'
        elif moyenne_seuil_min_rouge < area < moyenne_seuil_max_rouge:
            category = 'debout'
        else:
            category = 'allonge'

        # Dessiner le rectangle autour de l'objet rouge
        cv2.drawContours(result, [contour], 0, (0, 255, 255), 3)
        # Écrire la catégorie au centre de l'objet rouge
        cv2.putText(result, category, (x + int(w / 2), y + int(h / 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Incrémenter le compteur de zones bleues
        red_zones_count += 1


#Calculer le nombre de mannequins
nb_mannequins = blue_zones_count + red_zones_count

# Afficher le nombre de zones bleues identifiées
cv2.putText(result, f"Mannequins : {nb_mannequins}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
cv2.imwrite("Analyse_mannequins.jpg", result)
cv2.imwrite("Image_originale.jpg",image)
    
#Retour à la base
drone.set_mode("RTL")
