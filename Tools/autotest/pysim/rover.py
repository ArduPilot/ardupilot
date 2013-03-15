#!/usr/bin/env python
'''
simple rover simulator core
'''

from aircraft import Aircraft
import util, time, math
from math import degrees, radians
from rotmat import Vector3, Matrix3

class Rover(Aircraft):
    '''a simple rover'''
    def __init__(self,
                 max_speed=10,
                 max_accel=10,
                 max_turn_rate=45,
                 skid_steering=False):
        Aircraft.__init__(self)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_turn_rate = max_turn_rate
        self.last_time = time.time()
        self.skid_steering = skid_steering

    def update(self, state):

        # if in skid steering mode the steering and throttle values are used for motor1 and motor2
        if self.skid_steering:
            motor1 = state.steering # left motor
            motor2 = state.throttle # right motor
            steering = motor1 - motor2
            throttle = 0.5*(motor1 + motor2)
        else:
            steering = state.steering
            throttle = state.throttle

        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # speed in m/s in body frame
        velocity_body = self.dcm.transposed() * self.velocity

        # speed along x axis, +ve is forward
        speed = velocity_body.x

        # yaw rate in degrees/s
        yaw_rate = self.max_turn_rate * steering * (speed / self.max_speed)

        # target speed with current throttle
        target_speed = throttle * self.max_speed

        # linear acceleration in m/s/s - very crude model
        accel = self.max_accel * (target_speed - speed) / self.max_speed

#        print('speed=%f throttle=%f steering=%f yaw_rate=%f accel=%f' % (speed, state.throttle, state.steering, yaw_rate, accel))
        
        self.gyro = Vector3(0,0,radians(yaw_rate))

        # update attitude
        self.dcm.rotate(self.gyro * delta_time)
        self.dcm.normalize()

        # accel in body frame due to motor
        accel_body = Vector3(accel, 0, 0)

        # add in accel due to direction change
        accel_body.y += radians(yaw_rate) * speed

        # now in earth frame
        accel_earth = self.dcm * accel_body
        accel_earth += Vector3(0, 0, self.gravity)

        # if we're on the ground, then our vertical acceleration is limited
        # to zero. This effectively adds the force of the ground on the aircraft
        accel_earth.z = 0

        # work out acceleration as seen by the accelerometers. It sees the kinematic
        # acceleration (ie. real movement), plus gravity
        self.accel_body = self.dcm.transposed() * (accel_earth + Vector3(0, 0, -self.gravity))

        # new velocity vector
        self.velocity += accel_earth * delta_time

        # new position vector
        old_position = self.position.copy()
        self.position += self.velocity * delta_time

        # update lat/lon/altitude
        self.update_position(delta_time)

