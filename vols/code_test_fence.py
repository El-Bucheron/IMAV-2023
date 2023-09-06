# Code permettant d'importer la classe "Drone"
import os
import sys
package_path = os.getcwd()
while package_path[-9:] != "IMAV_2023":
    package_path = os.path.dirname(package_path)
sys.path.insert(0, package_path)

from Drone import Drone
from dronekit import LocationGlobalRelative
from pymavlink import mavutil
import time
drone_object = Drone()

#drone_object.list_fence()
#time.sleep(1)

#drone_object.load_fence()
#time.sleep(1)

#drone_object.send_fence()
#time.sleep(1)

#while True:
#    line = ""
#    for c in raw_input():
#        line = line + c
#    print("%s" % line)
#    if line == "exit":
#        break
while True:
  print("Waiting for BRAKE")
  if drone_object.get_mode() == "BRAKE":
    # print("Enter BRAKE mode")
    msg= drone_object.vehicle.message_factory.command_long_encode(
      # time_boot_ms (not used)
      0, 0,  # target system, target component
      mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,  # frame
      0, 1, 0, 0, 0, 0, 0, 0)
      # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
      #0, 0, 0,  # x, y, z acceleration (not used)
      #0, 0)  # yaw, yaw_rate (not used)
    
    # send command to vehicle
    #for x in range(0,duration) :
    drone_object.vehicle.send_mavlink(msg)
  time.sleep(0.5)
    # drone_object.set_mode("LAND")
