#!/usr/bin/env python

from aircraft import Aircraft
import euclid, util, time, math

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
                 hover_throttle=0.37,
                 terminal_velocity=30.0,
                 frame_height=0.1,
                 mass=1.0):
        Aircraft.__init__(self)
        self.motors = build_motors(frame)
        self.motor_speed = [ 0.0 ] * len(self.motors)
        self.mass = mass # Kg
        self.hover_throttle = hover_throttle
        self.terminal_velocity = terminal_velocity
        self.terminal_rotation_rate = 4*360.0
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

        # rotational acceleration, in degrees/s/s, in body frame
        roll_accel = 0.0
        pitch_accel = 0.0
        yaw_accel = 0.0
        thrust = 0.0
        for i in range(len(self.motors)):
            roll_accel  += -5000.0 * math.sin(math.radians(self.motors[i].angle)) * m[i]
            pitch_accel += 5000.0 * math.cos(math.radians(self.motors[i].angle)) * m[i]
            if self.motors[i].clockwise:
                yaw_accel -= m[i] * 400.0
            else:
                yaw_accel += m[i] * 400.0
            thrust += m[i] * self.thrust_scale # newtons

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

