import math, util, rotmat
from rotmat import Vector3, Matrix3

class Aircraft(object):
    '''a basic aircraft class'''
    def __init__(self):
        self.home_latitude = 0
        self.home_longitude = 0
        self.home_altitude = 0
        self.ground_level = 0
        self.frame_height = 0.0

        self.latitude  = self.home_latitude
        self.longitude = self.home_longitude
        self.altitude  = self.home_altitude

        self.dcm = Matrix3()

        # rotation rate in body frame
        self.gyro = Vector3(0,0,0) # rad/s

        self.velocity = Vector3(0, 0, 0) # m/s, North, East, Down
        self.position = Vector3(0, 0, 0) # m North, East, Down
        self.mass = 0.0
        self.update_frequency = 50 # in Hz
        self.gravity = 9.80665 # m/s/s
        self.accelerometer = Vector3(0, 0, -self.gravity)

        self.wind = util.Wind('0,0,0')

    def on_ground(self, position=None):
        '''return true if we are on the ground'''
        if position is None:
            position = self.position
        return (-position.z) + self.home_altitude <= self.ground_level + self.frame_height

    def update_position(self, delta_time):
        '''update lat/lon/alt from position'''

        bearing = math.degrees(math.atan2(self.position.y, self.position.x))
        distance = math.sqrt(self.position.x**2 + self.position.y**2)

        (self.latitude, self.longitude) = util.gps_newpos(self.home_latitude, self.home_longitude,
                                                          bearing, distance)

        self.altitude  = self.home_altitude - self.position.z

        velocity_body = self.dcm.transposed() * self.velocity

        self.accelerometer = self.accel_body.copy()

    def set_yaw_degrees(self, yaw_degrees):
        '''rotate to the given yaw'''
        (roll, pitch, yaw) = self.dcm.to_euler()
        yaw = math.radians(yaw_degrees)
        self.dcm.from_euler(roll, pitch, yaw)
        
