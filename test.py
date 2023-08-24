from commande_drone import *
from time import sleep
drone = Drone()

while True:
  aruco_center_x, aruco_center_y, _, image = drone.camera.detection_aruco(True)
  erreurX, erreurY, vx2, vy2 = drone.asservissement_suivi_vehicule_fonctionnel(aruco_center_x, aruco_center_y)
  print(str(vx2) + " " + str(vy2))
  sleep(2)