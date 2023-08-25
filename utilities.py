#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022
@author: Thomas Pavot
"""

import time
import os
import cv2
from dronekit import LocationGlobalRelative
from math import asin, atan2, cos, degrees, radians, sin, sqrt, pi
from datetime import datetime

R = 6371000 # Mean earth radius (meters)

def get_distance_metres(aLocation1, aLocation2):
    """
    Calculate distance in meters between Latitude/Longitude points.
    
    This uses the 'haversine' formula to calculate the great-circle
    distance between two points - that is, the shortest distance over
    the earth's surface earth's poles. More informations at:
    https://www.movable-type.co.uk/scripts/latlong.html
    """
    # Haversine formula to compute distance between two GPS points
    targLat = aLocation1.lat
    targLon = aLocation1.lon
    realLat = aLocation2.lat
    realLon = aLocation2.lon

    phi_1 = radians(targLat)
    phi_2 = radians(realLat)
    delta_phi = radians(targLat-realLat)    # Latitude difference (radians)
    delta_theta = radians(targLon-realLon)  # Longitude difference (radians)
    a = sin(delta_phi/2)**2 + cos(phi_1) * cos(phi_2) * sin(delta_theta/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    d = R * c

    return d



def get_GPS_location(aLocation, bearing, distance):
    """
    Calculate GPS target given distance and bearing from GPS start.

    Given a start point, initial bearing, and distance, this will
    calculate the destination point and travelling along a (shortest
    distance) great circle arc. More informations at:
    https://www.movable-type.co.uk/scripts/latlong.html
    """
    # Inverse of Haversine
    phi_1 = radians(aLocation.lat)
    lambda_1 = radians(aLocation.lon)
    phi_2 = asin(sin(phi_1) * cos(distance/R) + cos(phi_1) * sin(distance/R) * cos(bearing))
    lambda_2 = lambda_1 + atan2(sin(bearing) * sin(distance/R) * cos(phi_1), cos(distance/R) - sin(phi_1) * sin(phi_2))
    
    return LocationGlobalRelative(degrees(phi_2), degrees(lambda_2), 0)



def get_distance_angle_picture(x_image_center, y_image_center, x_target_center, y_target_center, altitude, dist_coeff_x, dist_coeff_y):
    """
    Calculate distance between two objects in a picture.

    Distances on x and y axes are dependant from sensor sizes and
    resolutions (which implies two different coefficients for each
    axis). More informations at:
    https://photo.stackexchange.com/questions/102795/calculate-the-distance-of-an-object-in-a-picture
    """
    if x_target_center == None:
        return None
    else:
        dist_x = altitude*(x_target_center - x_image_center)*dist_coeff_x
        dist_y = altitude*(y_target_center - y_image_center)*dist_coeff_y
        return sqrt(dist_x**2+dist_y**2), atan2(dist_y, dist_x)


# Fonction permettant de récupérer les coordonnées GPS d'un objet à partir des coordonnées X,Y d'une image
def get_GPS_through_picture(drone, X, Y):
    distance_vision, angle_vision = get_distance_angle_picture(
        drone.camera.x_imageCenter, drone.camera.y_imageCenter,
        X, Y,
        drone.vehicle.rangefinder.distance, drone.camera.dist_coeff_x, drone.camera.dist_coeff_y)
    current_location = LocationGlobalRelative(drone.vehicle.location.global_frame.lat, drone.vehicle.location.global_frame.lon, 0)
    estimated_location = get_GPS_location(current_location, drone.vehicle.attitude.yaw + angle_vision, distance_vision)
    return estimated_location


# Décorateur permettant d'afficher le temps d'excécution d'une fonction 
def get_excecution_time(function):
    def timer(*args, **kwargs):
        # Stockage du temps de départ de la fonction
        start_time = time.time()
        # Appel de la fonction
        result = function(*args, **kwargs)
        # Stockage du temps de fin de la fonction
        end_time = time.time()
        # Affichage du temps de calcul
        print(f'Temps d\'excécution "{function.__name__}" : {end_time-start_time:.4f} s')
        # revoi du résultat de la fonction appelée
        return result
    return timer 



# Cette fonction crée un dossier de nom "foldername" dans le dossier "photos" dans le dossier "IMAV_2023" et renvoie le chemin du dossier crée
# Elle ne fonctionne donc que si elle est appelée depuis une fonction située dans le dossier "IMAV_2023" ou un de ses sous-dossiers
def creation_dossier_photo(foldername):

    # Récupération du chemin du dossier "IMAV_2023"
    package_path = os.getcwd()
    while package_path[-9:] != "IMAV_2023":
        package_path = os.path.dirname(package_path)

    # Si le dossier "photos" n'existe pas on le crée
    if "photos" not in os.listdir(package_path):
        os.mkdir(os.path.join(package_path, "photos"))

    # On vérifie que le dossier n'existe pas déjà et on le crée si ce n'est pas le cas
    if foldername not in os.listdir(os.path.join(package_path, "photos")):
        os.mkdir(os.path.join(package_path, "photos", foldername))

    # Renvoi du chemin du dossier créé
    return os.path.join(package_path, "photos", foldername)



# Fonction permettant de sauvegarder une image "image" dans le dossier "folder_path" avec le format suivant :
# HH:MM:SS.MSS lattitude,longitude,atlitude complement_nom_fichier.jpg
# MSS représente les millisecondes de l'instant de l'enregistrement de la photo
def enregistrement_photo_date_position(drone, image, folder_path, complement_nom_fichier = ""):

    # Création du nom de la photo
    nom_photo = (datetime.now().strftime("%H:%M:%S.%f")[:-3] + " " +  # Heure de prise de la photo  
        str(drone.vehicle.location.global_relative_frame.lat) + "," + # Encodage de la Latitude
        str(drone.vehicle.location.global_relative_frame.lon) + "," + # Encodage de la longitude
        str('%.2f'%(drone.vehicle.rangefinder.distance)))             # Encodage de l'altitude
    # Ajout du complémet de nom et du format (.jpg) 
    nom_photo += (" " + complement_nom_fichier + ".jpg") if complement_nom_fichier != "" else ".jpg"
    
    # Écriture de l'image
    cv2.imwrite(os.path.join(folder_path, nom_photo), image)