#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022
@author: Thomas Pavot
"""
from dronekit import LocationGlobalRelative
from math import asin, atan2, cos, degrees, radians, sin, sqrt
import time

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
    # Input variables
    initLat = aLocation.lat
    initLon = aLocation.lon
    theta = bearing
    d = distance

    # Inverse of Haversine
    phi_1 = radians(initLat)
    lambda_1 = radians(initLon)
    phi_2 = asin(sin(phi_1) * cos(d/R) + cos(phi_1) * sin(d/R) * cos(theta))
    lambda_2 = lambda_1 + atan2(sin(theta) * sin(d/R) * cos(phi_1), cos(d/R) - sin(phi_1) * sin(phi_2))
    
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
        print(f'Temps d\'excécution "{function.__name__}" : {end_time-start_time:.4f} ms')
        # revoi du résultat de la fonction appelée
        return result
    return timer 
