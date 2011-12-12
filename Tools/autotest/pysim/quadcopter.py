#!/usr/bin/env python

from aircraft import Aircraft
import euclid, util, time


class QuadCopter(Aircraft):
    '''a QuadCopter'''
    def __init__(self):
        Aircraft.__init__(self)
        self.motor_speed = [ 0.0, 0.0, 0.0, 0.0 ]
        self.mass = 1.0 # Kg
        self.hover_throttle = 0.37
        self.terminal_velocity = 30.0
        self.terminal_rotation_rate = 4*360.0
        self.frame_height = 0.1

        # scaling from total motor power to Newtons. Allows the copter
        # to hover against gravity when each motor is at hover_throttle
        self.thrust_scale = (self.mass * self.gravity) / (4.0 * self.hover_throttle)

        self.last_time = time.time()

    def update(self, servos):
        for i in range(0, 4):
            if servos[i] <= 0.0:
                self.motor_speed[i] = 0
            else:
                self.motor_speed[i] = servos[i]

        m = self.motor_speed

        # how much time has passed?
        t = time.time()
        delta_time = t - self.last_time
        self.last_time = t

        # rotational acceleration, in degrees/s/s, in body frame
        roll_accel  = (m[1] - m[0]) * 5000.0
        pitch_accel = (m[2] - m[3]) * 5000.0
        yaw_accel   = -((m[2]+m[3]) - (m[0]+m[1])) * 400.0

        # rotational resistance
        roll_accel  -= (self.pDeg / self.terminal_rotation_rate) * 5000.0
        pitch_accel -= (self.qDeg / self.terminal_rotation_rate) * 5000.0
        yaw_accel   -= (self.rDeg / self.terminal_rotation_rate) * 400.0

        # update rotational rates in body frame
        self.pDeg  += roll_accel * delta_time
        self.qDeg  += pitch_accel * delta_time
        self.rDeg  += yaw_accel * delta_time

        # calculate rates in earth frame
        (self.roll_rate,
         self.pitch_rate,
         self.yaw_rate) =  util.BodyRatesToEarthRates(self.roll, self.pitch, self.yaw,
                                                      self.pDeg, self.qDeg, self.rDeg)

        # update rotation
        self.roll   += self.roll_rate  * delta_time
        self.pitch  += self.pitch_rate * delta_time
        self.yaw    += self.yaw_rate   * delta_time

        # air resistance
        air_resistance = - self.velocity * (self.gravity/self.terminal_velocity)

        # normalise rotations
        self.normalise()

        thrust = (m[0] + m[1] + m[2] + m[3]) * self.thrust_scale # Newtons
        accel = thrust / self.mass

        accel3D = util.RPY_to_XYZ(self.roll, self.pitch, self.yaw, accel)
        accel3D += euclid.Vector3(0, 0, -self.gravity)
        accel3D += air_resistance

        # add in some wind
        accel3D += self.wind.accel(self.velocity)

        # new velocity vector
        self.velocity += accel3D * delta_time
        self.accel     = accel3D

        # new position vector
        old_position = self.position.copy()
        self.position += self.velocity * delta_time

        # constrain height to the ground
        if self.position.z + self.home_altitude < self.ground_level + self.frame_height:
            if old_position.z + self.home_altitude > self.ground_level + self.frame_height:
                print("Hit ground at %f m/s" % (-self.velocity.z))
            self.velocity = euclid.Vector3(0, 0, 0)
            self.roll_rate = 0
            self.pitch_rate = 0
            self.yaw_rate = 0
            self.roll = 0
            self.pitch = 0
            self.accel = euclid.Vector3(0, 0, 0)
            self.position = euclid.Vector3(self.position.x, self.position.y,
                                           self.ground_level + self.frame_height - self.home_altitude)

        # update lat/lon/altitude
        self.update_position()
