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
        self.last_velocity = self.velocity.copy()
        self.mass = 0.0
        self.update_frequency = 50 # in Hz
        self.gravity = 9.8 # m/s/s
        self.accelerometer = Vector3(0, 0, self.gravity)

        self.wind = util.Wind('0,0,0')

    def update_position(self, delta_time):
        '''update lat/lon/alt from position'''

        radius_of_earth = 6378100.0 # in meters
        dlat = math.degrees(math.atan(self.position.x/radius_of_earth))
        self.latitude  = self.home_latitude + dlat
        lon_scale = math.cos(math.radians(self.latitude));
        dlon = math.degrees(math.atan(self.position.y/radius_of_earth))/lon_scale
        self.longitude = self.home_longitude + dlon

        self.altitude  = self.home_altitude - self.position.z

        # work out what the accelerometer would see
        self.accelerometer = ((self.velocity - self.last_velocity)/delta_time) + Vector3(0,0,self.gravity)
#        self.accelerometer = Vector3(0,0,-self.gravity)
        self.accelerometer = self.dcm.transposed() * self.accelerometer
        self.last_velocity = self.velocity.copy()
