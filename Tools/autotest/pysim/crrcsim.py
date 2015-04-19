#!/usr/bin/env python
'''
Python interface to CRRCSIM simulator
'''

from aircraft import Aircraft
import util, time, struct
import socket, select
from math import degrees, radians, sin, cos
from rotmat import Vector3, Matrix3

class CRRCSim(Aircraft):
    '''a CRRCCIM shim'''
    def __init__(self, frame="generic"):
        Aircraft.__init__(self)

        self.sim = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sim.connect(('127.0.0.1', 9002))
        self.sim.setblocking(0)
        self.accel_body = Vector3(0, 0, -self.gravity)
        self.frame = frame
        if self.frame.find("heli") != -1:
            self.decode_servos = self.decode_servos_heli
        else:
            self.decode_servos = self.decode_servos_fixed_wing
        self.rsc_setpoint = 0.8

    def decode_servos_heli(self, servos):
        '''decode servos for heli'''
        swash1 = servos[0]
        swash2 = servos[1]
        swash3 = servos[2]
        tail_rotor = servos[3]
        rsc = servos[7]
        rsc = util.constrain(rsc, 0, self.rsc_setpoint)

        # get total thrust from 0 to 1
        rsc_scaling = (rsc/self.rsc_setpoint)
        self.thrust = rsc_scaling*(swash1+swash2+swash3)/3.0

        # very simplistic mapping to body euler rates
        self.roll_rate = (swash1 - swash2)*rsc_scaling
        self.pitch_rate = -((swash1 + swash2)/2.0 - swash3)*rsc_scaling
        self.yaw_rate = -(tail_rotor - 0.5)*rsc_scaling

    def decode_servos_fixed_wing(self, servos):
        '''decode servos for fixed wing'''
        self.roll_rate = 2*(servos[0]-0.5)
        self.pitch_rate = 2*(servos[1]-0.5)
        self.thrust = servos[2]
        self.yaw_rate = 2*(servos[3]-0.5)
        

    def send_servos(self, servos):
        '''send servo packet'''
        self.decode_servos(servos)

        self.roll_rate = util.constrain(self.roll_rate, -1, 1)
        self.pitch_rate = util.constrain(self.pitch_rate, -1, 1)
        self.thrust = util.constrain(self.thrust, 0, 1)
        self.yaw_rate = util.constrain(self.yaw_rate, -1, 1)

        buf = struct.pack('<fffff',
                          self.roll_rate,
                          self.pitch_rate,
                          self.thrust,
                          self.yaw_rate,
                          0)
        self.sim.send(buf)

    def recv_fdm(self):
        '''receive FDM packet'''
        try:
            select.select([self.sim.fileno()], [], [], 1)
            buf = self.sim.recv(1024)
        except Exception as ex:
            return False

        (timestamp,
         latitude, longitude,
         altitude,
         heading,
         speedN, speedE, speedD,
         xAccel, yAccel, zAccel,
         rollRate, pitchRate, yawRate,
         roll, pitch, yaw,
         airspeed) = struct.unpack('<dddddddddddddddddd', buf)

        self.accel_body = Vector3(xAccel, yAccel, zAccel)
        self.gyro = Vector3(rollRate, pitchRate, yawRate)
        self.velocity = Vector3(speedN, speedE, speedD)
        dist = util.gps_distance(0, 0, latitude, longitude)
        bearing = util.gps_bearing(0, 0, latitude, longitude)
        self.position.x = dist * cos(radians(bearing))
        self.position.y = dist * sin(radians(bearing))
        self.position.z = -altitude
        self.dcm.from_euler(roll, pitch, yaw)
        self.time_now = timestamp + self.time_base

    def update(self, actuators):
        '''update model'''
        self.send_servos(actuators)
        self.recv_fdm()
        self.update_position()
