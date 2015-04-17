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
        self.last_fdm_time = time.time()
        self.frame = frame
        if self.frame.find("heli") != -1:
            self.decode_servos = self.decode_servos_heli
        else:
            self.decode_servos = self.decode_servos_fixed_wing
        self.rsc_setpoint = 0.8

    def checksum(self, buf):
        '''calculate MNAV checksum'''
        sum = 0
        for b in buf:
            sum += ord(b)
        return sum & 0xFFFF

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

        buf = struct.pack('>BBBBHHHH10x',
                          0x55, 0x55,
                          ord('S'), ord('S'),
                          int(self.roll_rate*32767)+32768,
                          int(self.pitch_rate*32767)+32768,
                          int(self.thrust*65535),
                          int(self.yaw_rate*32767)+32768)
        sum = self.checksum(buf[2:])
        buf += struct.pack('>H', sum)
        self.sim.send(buf)

    def recv_fdm(self):
        '''receive FDM packet'''
        try:
            select.select([self.sim.fileno()], [], [], 1)
            buf = self.sim.recv(1024)
            while True:
                try:
                    buf2 = self.sim.recv(1024)
                    buf = buf2
                except Exception:
                    break
        except Exception as ex:
            return False

        # the MNAV packets come as 3 chunks, called IMU, GPS and AHRS
        if len(buf) != 93 or buf[0:3] != 'UUI' or buf[33] != 'G' or buf[66] != 'A':
            return False

        # decode IMU chunk
        ax,ay,az = struct.unpack(">hhh", buf[3:9])
        self.accel_body = Vector3(ax, ay, az) * 5.98755e-04

        gx,gy,gz = struct.unpack(">hhh", buf[9:15])
        self.gyro = Vector3(gx, gy, gz) * 1.06526e-04

        # decode GPS chunk
        vx,vy,vz = struct.unpack("<iii", buf[34:46])
        self.velocity = Vector3(vx,vy,vz) * 1.0e-2

        lon,lat,alt = struct.unpack('<iii', buf[46:58])
        lat *= 1.0e-7
        lon *= 1.0e-7
        alt *= 1.0e-3
        dist = util.gps_distance(0, 0, lat, lon)
        bearing = util.gps_bearing(0, 0, lat, lon)
        self.position.x = dist * cos(radians(bearing))
        self.position.y = dist * sin(radians(bearing))
        self.position.z = -alt

        # decode AHRS chunk
        roll,pitch,yaw = struct.unpack(">hhh", buf[67:73])
        self.dcm.from_euler(roll*0.000106526, pitch*0.000106526, yaw*0.000106526)

        now = time.time()
        self.last_fdm_time = now

    def update(self, actuators):
        '''update model'''
        self.send_servos(actuators)
        self.recv_fdm()
        self.update_position()
