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
        self.gravity = 9.8 # m/s/s
        self.accelerometer = Vector3(0, 0, -self.gravity)

        self.wind = util.Wind('0,0,0')

    def on_ground(self, position=None):
        '''return true if we are on the ground'''
        if position is None:
            position = self.position
        return (-position.z) + self.home_altitude <= self.ground_level + self.frame_height


    def update_position(self, delta_time):
        '''update lat/lon/alt from position'''

        radius_of_earth = 6378100.0 # in meters
        dlat = math.degrees(math.atan(self.position.x/radius_of_earth))
        self.latitude  = self.home_latitude + dlat
        lon_scale = math.cos(math.radians(self.latitude));
        dlon = math.degrees(math.atan(self.position.y/radius_of_earth))/lon_scale
        self.longitude = self.home_longitude + dlon

        self.altitude  = self.home_altitude - self.position.z

        velocity_body = self.dcm.transposed() * self.velocity

        # force the acceleration to mostly be from gravity. We should be using 100% accel_body,
        # but right now that flies very badly as the AHRS system can't do centripetal correction
        # for multicopters. This is a compromise until we get that sorted out
        accel_true = self.accel_body
        accel_fake = self.dcm.transposed() * Vector3(0, 0, -self.gravity)
        self.accelerometer = (accel_true * 0.5) + (accel_fake * 0.5)
