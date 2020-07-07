import Drone
import time









drone = Drone.Tello()
drone.AIRSIM_ENABLED = False
drone.TELLO_ENABLED = True

drone.connect()

drone.get_battery_status()
drone.get_speed()
drone.takeoff()
drone.forward(50)
drone.left(50)
drone.rotate(90)
drone.forward(50)
drone.rotate(90)
drone.forward(50)
drone.rotate(90)
drone.forward(50)



