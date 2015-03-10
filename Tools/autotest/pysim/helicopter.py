#!/usr/bin/env python

from aircraft import Aircraft
import util, time, math
from math import degrees, radians
from rotmat import Vector3, Matrix3

class HeliCopter(Aircraft):
    '''a HeliCopter simulator'''
    def __init__(self, frame='+',
                 hover_throttle=0.65,
                 terminal_velocity=40.0,
                 frame_height=0.1,
                 mass=2.5):
        Aircraft.__init__(self)
        self.mass = mass # Kg
        self.hover_throttle = hover_throttle
        self.terminal_velocity = terminal_velocity
        self.terminal_rotation_rate = 4*radians(360.0)
        self.frame_height = frame_height
        self.last_time = time.time()
        self.roll_rate_max = radians(720)
        self.pitch_rate_max = radians(720)
        self.yaw_rate_max = radians(720)
        self.rsc_setpoint = 0.8
        self.thrust_scale = (self.mass * self.gravity) / self.hover_throttle

    def update(self, servos):
        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # get swash proportions (these are from 0 to 1)
        swash1 = servos[0]
        swash2 = servos[1]
        swash3 = servos[2]
        tail_rotor = servos[3]
        rsc = servos[7]

        # get total thrust from 0 to 1
        thrust = (rsc/self.rsc_setpoint)*(swash1+swash2+swash3)/3.0

        # very simplistic mapping to body euler rates
        roll_rate = swash1 - swash2
        pitch_rate = (swash1 + swash2)/2.0 - swash3
        yaw_rate = tail_rotor - 0.5

        #print(swash1, swash2, swash3, roll_rate, pitch_rate, yaw_rate, thrust, rsc)

        roll_rate *= rsc/self.rsc_setpoint
        pitch_rate *= rsc/self.rsc_setpoint
        yaw_rate *= rsc/self.rsc_setpoint

        rot_accel = Vector3(roll_rate*self.roll_rate_max,
                            pitch_rate*self.pitch_rate_max,
                            yaw_rate*self.yaw_rate_max)

        # rotational air resistance
        rot_accel.x -= self.gyro.x * radians(5000.0) / self.terminal_rotation_rate
        rot_accel.y -= self.gyro.y * radians(5000.0) / self.terminal_rotation_rate
        rot_accel.z -= self.gyro.z * radians(5000.0) / self.terminal_rotation_rate

        # update rotational rates in body frame
        self.gyro += rot_accel * delta_time

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # air resistance
        air_resistance = - self.velocity * (self.gravity/self.terminal_velocity)

        thrust *= self.thrust_scale

        accel_body = Vector3(0, 0, -thrust / self.mass)
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)
        accel_earth += air_resistance

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
