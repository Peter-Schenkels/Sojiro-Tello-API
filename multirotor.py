import math
import time
import airsim

class Multirotor():



    def __init__(self, name):
        """constructor MPLA Airsim

        :param name: drone name in settings.json
        """
        self.client = airsim.MultirotorClient()
        self.name = name
        # Speed in velocity percentage
        self.speed = 1.00
        # Maximum velocity
        self.max_velocity = 1
        # Turning speed
        self.degrees_per_second = 45
        # Delay in seconds between every operation
        self.delay = 1

    def takeoff(self):
        self.client.takeoffAsync(timeout_sec=1, vehicle_name=self.name).join()

    def get_speed(self):
        """returns the setted movement speed of the Multirotor.

        :return: velocity in percentage
        """
        return self.speed

    def set_speed(self, speed_percentage):
        """ Set velocity percentage

        :param speed_percentage: percentage of the max velocity
        """
        if speed_percentage > 100:
            self.speed = 100
        else:
            self.speed = speed_percentage * (self.max_velocity / 100)

    def set_degrees_per_second(self, degrees):
        """This function sets the rotation speed in degrees per second.

        This function sets the multirotor rotation speed in degrees per second, the given degrees per second
        can be infinite but is not recommended any higher than 180 degrees per second.

        :param degrees: degrees per second.
        """
        self.degrees_per_second = degrees

    def get_position(self):
        """Get drone position

        :return: Drone position
        """
        return self.client.getMultirotorState(vehicle_name=self.name).kinematics_estimated.position

    def get_degrees_per_second(self):
        """This function returns the rotation speed in degrees per second.

        :return: degrees per second
        """
        return self.degrees_per_second

    def turn(self, degrees):
        """This function will rotate the UAV in the given degrees.

        :param: degrees in degrees.
        """
        new_degrees_per_second = 0

        if degrees < self.degrees_per_second and degrees > -self.degrees_per_second:
            new_degrees_per_second = degrees
            rate = 1

        elif degrees % self.degrees_per_second == 0:
            if degrees < 0:
                new_degrees_per_second = -self.degrees_per_second

            else:
                new_degrees_per_second = self.degrees_per_second

            rate = abs(int(degrees / self.degrees_per_second))

        else:
            rate = abs(int(degrees / self.degrees_per_second))
            overhead = (degrees % self.degrees_per_second) / rate
            if degrees > 0:
                new_degrees_per_second = self.degrees_per_second + overhead
            else:
                new_degrees_per_second = -self.degrees_per_second - overhead

        self.client.rotateByYawRateAsync(new_degrees_per_second, rate, vehicle_name=self.name).join()
        time.sleep(self.delay)

    def get_orientation(self):
        """
        :return:
        """
        return airsim.to_eularian_angles(
            self.client.getMultirotorState(vehicle_name=self.name).kinematics_estimated.orientation)

    def move(self, distance):
        """This function will move the platform a given distance in m.

        :param distance in meters.
        """
        yaw = (
            airsim.to_eularian_angles(
                self.client.getMultirotorState(vehicle_name=self.name).kinematics_estimated.orientation)[2])
        position = self.get_position()
        offset = list(self.calculate_offset(distance, yaw))
        self.move_a_to_b(
            airsim.Vector3r(position.x_val + offset[0],
            position.y_val + offset[1],
            position.z_val))
        time.sleep(self.delay)

    @staticmethod
    def make_position_relative(self, x, y, z):
        """This function will adjust the position of a point to relative to the rotation of the multirotor

        @param x : Relative to world x position.
        @param y : Relative to world y position.
        @param z : Relative to world z position.
        """
        position = self.client.getMultirotorState(vehicle_name=self.name).kinematics_estimated.position
        x -= position.x_val
        y -= position.y_val
        z -= position.z_val
        return airsim.Vector3r(x, y, z)


    def  move_a_to_b(self, destination : airsim.Vector3r):
        """This function will move the platform to a given position.

        :param destination : Relative to world x, y, z position.
        """
        delta = 0.1
        while abs(self.get_position().x_val - destination.x_val) > delta or abs(self.get_position().y_val - destination.y_val ) > delta or abs(self.get_position().z_val - destination.z_val) > delta:
            y_velocity = 0
            x_velocity = 0

            delta_x = destination.x_val - self.get_position().x_val
            delta_y = destination.y_val - self.get_position().y_val
            delta_z = destination.z_val - self.get_position().z_val

            x_velocity = delta_x / (math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2) / (self.speed * self.max_velocity))
            y_velocity = delta_y / (math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2) / (self.speed * self.max_velocity))
            z_velocity = delta_z / (math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2) / (self.speed * self.max_velocity))

            self.client.moveByVelocityAsync(x_velocity, y_velocity, z_velocity, duration=0.01, vehicle_name=self.name)

    @staticmethod
    def calculate_offset(distance, angle):
        """Basic trigonometry function.

        :param distance : The sloping side of the triangle.
        :param angle : The angle of the sloping side in radians.
        """
        x_offset = round(distance * math.cos(angle), 1)
        y_offset = round(distance * math.sin(angle), 1)
        return x_offset, y_offset

    def connect(self):
        """Connect to airsim api.

        Confirms the connection, enables the api control, arms the drone and
        initiates takeoff.
        """
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=self.name)
        self.client.armDisarm(True, vehicle_name=self.name)


    def disconnect(self):
        """Disconnect from airsim api

        Disarms the drone, resets the position of the drone and disables the api
        control.
        """
        self.client.armDisarm(False, vehicle_name=self.name)
        self.client.reset()
        self.client.enableApiControl(False, vehicle_name=self.name)

    # @Brief

    def move_z(self, distance):
        """Move UAV x amount of meters upwards.

        Move the multirotor upwards in x amount of meters asynchronously.
        :param distance : amount of meters the drone should move upwards.
        """
        position = self.get_position()
        self.client.moveToPositionAsync(
            position.x_val,
            position.y_val,
            position.z_val - distance,
            self.speed * self.max_velocity, vehicle_name=self.name).join()

    def move_sideways(self, distance):
        """Move UAV x amount of meters sideways.

        Move the multirotor sideways in x amount of meters asynchronously.
        :param distance : amount of meters the drone should move sideways, positive is left negative is right.
        """
        position = self.get_position()
        offset = list(self.calculate_offset(distance, self.get_orientation()[2] - 1.5707963268))
        self.move_a_to_b(airsim.Vector3r(
            position.x_val + offset[0],
            position.y_val + offset[1],
            position.z_val))
        time.sleep(self.delay)
