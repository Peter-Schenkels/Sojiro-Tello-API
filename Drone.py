import threading
import socket
from djitellopy import Tello
import time
import multirotor


class Vector3:
    def __int__(self):
        self.x = 0
        self.y = 0
        self.z = 0

    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z

    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z

    def __mul__(self, other):
        self.x *= other.x
        self.y *= other.y
        self.z *= other.z



class Tello:
    def __init__(self):

        self.HOST = "192.168.10.1"
        self.PORT = 8889
        self.ADDRESS = (self.HOST, self.PORT)

        self.airsim_client = multirotor.Multirotor("Drone1")

        self.response = None
        self.TIME_OUT = 1
        self.TIME_OUT_COMMAND = 0
        self.speed = 0

        self.AIRSIM_ENABLED= False
        self.TELLO_ENABLED = True

        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client.bind(('', self.PORT))

        listen_for_response = threading.Thread(target=self.response_listener, args=())

        listen_for_response.daemon = True
        listen_for_response.start()



    def response_listener(self):
        while True:
            try:
                self.response = self.client.recv(1024).decode('utf-8')
            except Exception as e:
                print(e)

    def response_timeout(self):
        current_time = time.time()
        while(current_time + self.TIME_OUT > time.time()):
            if self.response != None:
                return True
        print("Error timed out")
        return False

    def send_command(self, command):
        print("send command,", command)
        self.tello_send_command(command)

    def tello_send_command(self, command):
        self.client.sendto(command.encode('utf-8'), self.ADDRESS)
        if self.response_timeout():
            print(self.response)
            self.response = None
        while (time.time() < self.TIME_OUT_COMMAND):
            None


    def connect(self):
        if(self.TELLO_ENABLED):
            self.send_command("command")
        if(self.AIRSIM_ENABLED):
            self.airsim_client.connect()


    def move(self, direction, distance):
        self.send_command(direction + ' ' + str(distance))
        self.TIME_OUT_COMMAND = time.time() + (distance / self.get_speed())*2

    def get_speed(self):
        self.send_command("speed?")
        if self.response != None:
            return float(self.response)
        else:
            return 5

    def get_battery_status(self):
        self.send_command("battery?")

    def takeoff(self):
        self.TIME_OUT_COMMAND = time.time() + 10
        if(self.TELLO_ENABLED):
            self.send_command("takeoff")
        if(self.AIRSIM_ENABLED):
            self.airsim_client.takeoff()

    def forward(self, distance: int):
        if(self.TELLO_ENABLED):
            self.move("forward", distance)
        if(self.AIRSIM_ENABLED):
            self.airsim_client.move(distance / 100)

    def backward(self, distance: int):
        if(self.TELLO_ENABLED):
            self.move("back", distance)
        if(self.AIRSIM_ENABLED):
            self.airsim_client.move(-distance / 100)

    def left(self, distance):
        if(self.TELLO_ENABLED):
            self.move("left", distance)
        if(self.AIRSIM_ENABLED):
            self.airsim_client.move_sideways(-distance / 100)

    def right(self, distance):
        if(self.TELLO_ENABLED):
            self.move("right", distance)
        if(self.AIRSIM_ENABLED):
            self.airsim_client.move_sideways(-distance / 100)

    def land(self):
        self.send_command("land")

    def rotate(self, degrees):
        self.TIME_OUT_COMMAND = time.time() + 3*degrees/45
        if(self.TELLO_ENABLED):
            if(degrees > 0):
                self.send_command("cw " + str(degrees))
            else:
                self.send_command("ccw" + str(degrees))
        if(self.AIRSIM_ENABLED):
            self.airsim_client.turn(degrees)