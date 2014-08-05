#!/usr/bin/env python

from aircraft import Aircraft
import util, time, math
from math import degrees, radians
from rotmat import Vector3, Matrix3

class Motor(object):
    def __init__(self, angle, clockwise, servo):
        self.angle = angle # angle in degrees from front
        self.clockwise = clockwise # clockwise == true, anti-clockwise == false
        self.servo = servo # what servo output drives this motor


def build_motors(frame):
    '''build a motors list given a frame type'''
    frame = frame.lower()
    if frame in [ 'quad', '+', 'x' ]:
        motors = [
            Motor(90,  False,  1),
            Motor(270, False,  2),
            Motor(0,   True,   3),
            Motor(180, True,   4),
            ]
        if frame in [ 'x', 'quadx' ]:
            for i in range(4):
                motors[i].angle -= 45.0
    elif frame in ["y6"]:
        motors = [
            Motor(60,   False, 1),
            Motor(60,   True,  7),
            Motor(180,  True,  4),
            Motor(180,  False, 8),
            Motor(-60,  True,  2),
            Motor(-60,  False, 3),
            ]
    elif frame in ["hexa", "hexa+"]:
        motors = [
            Motor(0,   True,  1),
            Motor(60,  False, 4),
            Motor(120, True,  8),
            Motor(180, False, 2),
            Motor(240, True,  3),
            Motor(300, False, 7),
            ]
    elif frame in ["hexax"]:
        motors = [
            Motor(30,  False,  7),
            Motor(90,  True,   1),
            Motor(150, False,  4),
            Motor(210, True,   8),
            Motor(270, False,  2),
            Motor(330, True,   3),
            ]
    elif frame in ["octa", "octa+", "octax" ]:
        motors = [
            Motor(0,    True,  1),
            Motor(180,  True,  2),
            Motor(45,   False, 3),
            Motor(135,  False, 4),
            Motor(-45,  False, 7),
            Motor(-135, False, 8),
            Motor(270,  True, 10),
            Motor(90,   True, 11),
            ]
        if frame == 'octax':
            for i in range(8):
                motors[i].angle += 22.5
    else:
        raise RuntimeError("Unknown multicopter frame type '%s'" % frame)

    return motors


class MultiCopter(Aircraft):
    '''a MultiCopter'''
    def __init__(self, frame='+',
                 hover_throttle=0.45,
                 terminal_velocity=15.0,
                 frame_height=0.1,
                 mass=1.5):
        Aircraft.__init__(self)
        self.motors = build_motors(frame)
        self.motor_speed = [ 0.0 ] * len(self.motors)
        self.mass = mass # Kg
        self.hover_throttle = hover_throttle
        self.terminal_velocity = terminal_velocity
        self.terminal_rotation_rate = 4*radians(360.0)
        self.frame_height = frame_height

        # scaling from total motor power to Newtons. Allows the copter
        # to hover against gravity when each motor is at hover_throttle
        self.thrust_scale = (self.mass * self.gravity) / (len(self.motors) * self.hover_throttle)

        self.last_time = time.time()

    def update(self, servos):
        for i in range(0, len(self.motors)):
            servo = servos[self.motors[i].servo-1]
            if servo <= 0.0:

                self.motor_speed[i] = 0
            else:
                self.motor_speed[i] = servo

        m = self.motor_speed

        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # rotational acceleration, in rad/s/s, in body frame
        rot_accel = Vector3(0,0,0)
        thrust = 0.0
        for i in range(len(self.motors)):
            rot_accel.x  += -radians(5000.0) * math.sin(radians(self.motors[i].angle)) * m[i]
            rot_accel.y  +=  radians(5000.0) * math.cos(radians(self.motors[i].angle)) * m[i]
            if self.motors[i].clockwise:
                rot_accel.z -= m[i] * radians(400.0)
            else:
                rot_accel.z += m[i] * radians(400.0)
            thrust += m[i] * self.thrust_scale # newtons

        # rotational air resistance
        rot_accel.x -= self.gyro.x * radians(5000.0) / self.terminal_rotation_rate
        rot_accel.y -= self.gyro.y * radians(5000.0) / self.terminal_rotation_rate
        rot_accel.z -= self.gyro.z * radians(400.0)  / self.terminal_rotation_rate

        # update rotational rates in body frame
        self.gyro += rot_accel * delta_time

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # air resistance
        air_resistance = - self.velocity * (self.gravity/self.terminal_velocity)

        accel_body = Vector3(0, 0, -thrust / self.mass)
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)
        accel_earth += air_resistance

        # add in some wind (turn force into accel by dividing by mass).
        # NOTE: disable this drag correction until we work out
        # why it is blowing up
        # accel_earth += self.wind.drag(self.velocity) / self.mass

        # if we're on the ground, then our vertical acceleration is limited
        # to zero. This effectively adds the force of the ground on the aircraft
        if self.on_ground() and accel_earth.z > 0:
            accel_earth.z = 0

        # work out acceleration as seen by the accelerometers. It sees the kinematic
        # acceleration (ie. real movement), plus gravity
        self.accel_body = self.dcm.transposed() * (accel_earth + Vector3(0, 0, -self.gravity))

        # new velocity vector
        self.velocity += accel_earth * delta_time

        # new position vector
        old_position = self.position.copy()
        self.position += self.velocity * delta_time

        # constrain height to the ground
        if self.on_ground():
            if not self.on_ground(old_position):
                print("Hit ground at %f m/s" % (self.velocity.z))

            self.velocity = Vector3(0, 0, 0)
            # zero roll/pitch, but keep yaw
            (r, p, y) = self.dcm.to_euler()
            self.dcm.from_euler(0, 0, y)

            self.position = Vector3(self.position.x, self.position.y,
                                    -(self.ground_level + self.frame_height - self.home_altitude))

        # update lat/lon/altitude
        self.update_position(delta_time)
