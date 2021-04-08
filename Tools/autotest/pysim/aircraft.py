import math
import random
import time
import util

from pymavlink.rotmat import Vector3, Matrix3


class Aircraft(object):
    """A basic aircraft class."""
    def __init__(self):
        self.home_latitude = 0
        self.home_longitude = 0
        self.home_altitude = 0
        self.ground_level = 0
        self.frame_height = 0.0

        self.latitude = self.home_latitude
        self.longitude = self.home_longitude
        self.altitude = self.home_altitude

        self.dcm = Matrix3()

        # rotation rate in body frame
        self.gyro = Vector3(0, 0, 0)  # rad/s

        self.velocity = Vector3(0, 0, 0)  # m/s, North, East, Down
        self.position = Vector3(0, 0, 0)  # m North, East, Down
        self.mass = 0.0
        self.update_frequency = 50  # in Hz
        self.gravity = 9.80665  # m/s/s
        self.accelerometer = Vector3(0, 0, -self.gravity)

        self.wind = util.Wind('0,0,0')
        self.time_base = time.time()
        self.time_now = self.time_base + 100*1.0e-6

        self.gyro_noise = math.radians(0.1)
        self.accel_noise = 0.3

    def on_ground(self, position=None):
        """Return true if we are on the ground."""
        if position is None:
            position = self.position
        return (-position.z) + self.home_altitude <= self.ground_level + self.frame_height

    def update_position(self):
        """Update lat/lon/alt from position."""

        bearing = math.degrees(math.atan2(self.position.y, self.position.x))
        distance = math.sqrt(self.position.x**2 + self.position.y**2)

        (self.latitude, self.longitude) = util.gps_newpos(self.home_latitude, self.home_longitude,
                                                          bearing, distance)

        self.altitude = self.home_altitude - self.position.z

        velocity_body = self.dcm.transposed() * self.velocity

        self.accelerometer = self.accel_body.copy()

    def set_yaw_degrees(self, yaw_degrees):
        """Rotate to the given yaw."""
        (roll, pitch, yaw) = self.dcm.to_euler()
        yaw = math.radians(yaw_degrees)
        self.dcm.from_euler(roll, pitch, yaw)

    def time_advance(self, deltat):
        """Advance time by deltat in seconds."""
        self.time_now += deltat

    def setup_frame_time(self, rate, speedup):
        """Setup frame_time calculation."""
        self.rate = rate
        self.speedup = speedup
        self.frame_time = 1.0/rate
        self.scaled_frame_time = self.frame_time/speedup
        self.last_wall_time = time.time()
        self.achieved_rate = rate

    def adjust_frame_time(self, rate):
        """Adjust frame_time calculation."""
        self.rate = rate
        self.frame_time = 1.0/rate
        self.scaled_frame_time = self.frame_time/self.speedup

    def sync_frame_time(self):
        """Try to synchronise simulation time with wall clock time, taking
        into account desired speedup."""
        now = time.time()
        if now < self.last_wall_time + self.scaled_frame_time:
            time.sleep(self.last_wall_time+self.scaled_frame_time - now)
            now = time.time()

        if now > self.last_wall_time and now - self.last_wall_time < 0.1:
            rate = 1.0/(now - self.last_wall_time)
            self.achieved_rate = (0.98*self.achieved_rate) + (0.02*rate)
            if self.achieved_rate < self.rate*self.speedup:
                self.scaled_frame_time *= 0.999
            else:
                self.scaled_frame_time *= 1.001

        self.last_wall_time = now

    def add_noise(self, throttle):
        """Add noise based on throttle level (from 0..1)."""
        self.gyro += Vector3(random.gauss(0, 1),
                             random.gauss(0, 1),
                             random.gauss(0, 1)) * throttle * self.gyro_noise
        self.accel_body += Vector3(random.gauss(0, 1),
                                   random.gauss(0, 1),
                                   random.gauss(0, 1)) * throttle * self.accel_noise
