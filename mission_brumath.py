from commande_drone import Drone
drone = Drone()
while drone.get_mode() != "AUTO":
    pass
drone.set_mode("GUIDED")
drone.arm_and_takeoff(10)
point = LocationGlobalRelative(48.706580, 7.734317, 10)
drone.goto(point, 1)
point = LocationGlobalRelative(48.706499, 7.734165, 10)
drone.goto(point, 1)
point = LocationGlobalRelative(48.706411, 7.734345, 10)
drone.goto(point, 1)
point = LocationGlobalRelative(48.706502 7.734511, 10)
drone.goto(point, 1)