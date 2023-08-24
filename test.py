from commande_drone import *
from time import sleep
from dronekit import LocationGlobalRelative

drone = Drone()
while True:
    print(drone.vehicle.location._lat)
    sleep(1)