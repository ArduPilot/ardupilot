import euclid, math, util


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

        self.pitch = 0.0        # degrees
        self.roll = 0.0         # degrees
        self.yaw = 0.0          # degrees

        # rates in earth frame
        self.pitch_rate = 0.0   # degrees/s
        self.roll_rate = 0.0    # degrees/s
        self.yaw_rate = 0.0     # degrees/s

        # rates in body frame
        self.pDeg = 0.0   # degrees/s
        self.qDeg = 0.0   # degrees/s
        self.rDeg = 0.0   # degrees/s

        self.velocity = euclid.Vector3(0, 0, 0) # m/s, North, East, Up
        self.position = euclid.Vector3(0, 0, 0) # m North, East, Up
        self.accel    = euclid.Vector3(0, 0, 0) # m/s/s North, East, Up
        self.mass = 0.0
        self.update_frequency = 50 # in Hz
        self.gravity = 9.8 # m/s/s
        self.accelerometer = euclid.Vector3(0, 0, -self.gravity)

        self.wind = util.Wind('0,0,0')

    def normalise(self):
        '''normalise roll, pitch and yaw

        roll between -180 and 180
        pitch between -180 and 180
        yaw between 0 and 360

        '''
        def norm(angle, min, max):
            while angle > max:
                angle -= 360
            while angle < min:
                angle += 360
            return angle
        self.roll  = norm(self.roll, -180, 180)
        self.pitch = norm(self.pitch, -180, 180)
        self.yaw   = norm(self.yaw, 0, 360)

    def update_position(self):
        '''update lat/lon/alt from position'''

        radius_of_earth = 6378100.0 # in meters
        dlat = math.degrees(math.atan(self.position.x/radius_of_earth))
        dlon = math.degrees(math.atan(self.position.y/radius_of_earth))

        self.altitude  = self.home_altitude + self.position.z
        self.latitude  = self.home_latitude + dlat
        self.longitude = self.home_longitude + dlon

        from math import sin, cos, sqrt, radians
        
        # work out what the accelerometer would see
        xAccel = sin(radians(self.pitch)) * cos(radians(self.roll))
        yAccel = -sin(radians(self.roll)) * cos(radians(self.pitch))
        zAccel = -cos(radians(self.roll)) * cos(radians(self.pitch))
        scale = 9.81 / sqrt((xAccel*xAccel)+(yAccel*yAccel)+(zAccel*zAccel))
        xAccel *= scale;
        yAccel *= scale;
        zAccel *= scale;
        self.accelerometer = euclid.Vector3(xAccel, yAccel, zAccel)
