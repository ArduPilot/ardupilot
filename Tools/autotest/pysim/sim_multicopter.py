#!/usr/bin/env python

from multicopter import MultiCopter
from helicopter import HeliCopter
import util, time, os, sys, math
import socket, struct
import select, errno

from pymavlink import fgFDM

def sim_send(m, a):
    '''send flight information to mavproxy and flightgear'''
    global fdm
    from math import degrees

    earth_rates = util.BodyRatesToEarthRates(a.dcm, a.gyro)
    (roll, pitch, yaw) = a.dcm.to_euler()

    fdm.set('latitude', a.latitude, units='degrees')
    fdm.set('longitude', a.longitude, units='degrees')
    fdm.set('altitude', a.altitude, units='meters')
    fdm.set('phi', roll, units='radians')
    fdm.set('theta', pitch, units='radians')
    fdm.set('psi', yaw, units='radians')
    fdm.set('phidot', earth_rates.x, units='rps')
    fdm.set('thetadot', earth_rates.y, units='rps')
    fdm.set('psidot', earth_rates.z, units='rps')
    fdm.set('vcas', math.sqrt(a.velocity.x*a.velocity.x + a.velocity.y*a.velocity.y), units='mps')
    fdm.set('v_north', a.velocity.x, units='mps')
    fdm.set('v_east', a.velocity.y, units='mps')
    # FG FDM protocol only supports 4 motors for display :(
    fdm.set('num_engines', 4)
    for i in range(4):
        fdm.set('rpm', 1000*m[i], idx=i)
    try:
        fg_out.send(fdm.pack())
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise

    timestamp = int((a.time_now - a.time_base) * 1e6)

    buf = struct.pack('<Q17dI',
                      timestamp,
                      a.latitude, a.longitude, a.altitude, degrees(yaw),
                      a.velocity.x, a.velocity.y, a.velocity.z,
                      a.accelerometer.x, a.accelerometer.y, a.accelerometer.z,
                      degrees(earth_rates.x), degrees(earth_rates.y), degrees(earth_rates.z),
                      degrees(roll), degrees(pitch), degrees(yaw),
                      math.sqrt(a.velocity.x*a.velocity.x + a.velocity.y*a.velocity.y),
                      0x4c56414f)
    try:
        sim_out.send(buf)
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise


def sim_recv(m):
    '''receive control information from SITL'''
    try:
        buf = sim_in.recv(28)
    except socket.error as e:
        if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            raise
        return

    if len(buf) != 28:
        return
    control = list(struct.unpack('<14H', buf))
    pwm = control[0:11]

    # update motors
    for i in range(11):
        m[i] = (pwm[i]-1000)/1000.0

    # update wind
    global a
    (speed, direction, turbulance) = control[11:]
    a.wind.speed = speed*0.01
    a.wind.direction = direction*0.01
    a.wind.turbulance = turbulance*0.01
    


def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

##################
# main program
from optparse import OptionParser
parser = OptionParser("sim_multicopter.py [options]")
parser.add_option("--fgout", dest="fgout",  help="flightgear output (IP:port)", default="127.0.0.1:5503")
parser.add_option("--simin",  dest="simin",   help="SIM input (IP:port)",       default="127.0.0.1:5502")
parser.add_option("--simout", dest="simout",  help="SIM output (IP:port)",      default="127.0.0.1:5501")
parser.add_option("--home", dest="home",  type='string', default=None, help="home lat,lng,alt,hdg (required)")
parser.add_option("--rate", dest="rate", type='int', help="SIM update rate", default=400)
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0')
parser.add_option("--frame", dest="frame", help="frame type (+,X,octo)", default='+')
parser.add_option("--gimbal", dest="gimbal", action='store_true', default=False, help="enable gimbal")
parser.add_option("--nowait", action='store_true', help="don't pause between updates")

(opts, args) = parser.parse_args()

for m in [ 'home' ]:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)
        parser.print_help()
        sys.exit(1)

# UDP socket addresses
fg_out_address  = interpret_address(opts.fgout)
sim_out_address = interpret_address(opts.simout)
sim_in_address  = interpret_address(opts.simin)

# setup output to flightgear
fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fg_out.connect(fg_out_address)
fg_out.setblocking(0)

# setup input from SITL
sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_in.bind(sim_in_address)
sim_in.setblocking(1)

# setup output to SITL
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(sim_out_address)
sim_out.setblocking(0)

# FG FDM object
fdm = fgFDM.fgFDM()

# create the quadcopter model
if opts.frame == 'heli':
    a = HeliCopter(frame=opts.frame)
else:    
    a = MultiCopter(frame=opts.frame)

print("Simulating for frame %s" % opts.frame)

# motors initially off
m = [0.0] * 11

# parse home
v = opts.home.split(',')
if len(v) != 4:
    print("home should be lat,lng,alt,hdg")
    sys.exit(1)
a.home_latitude = float(v[0])
a.home_longitude = float(v[1])
a.home_altitude = float(v[2])
a.latitude = a.home_latitude
a.longitude = a.home_longitude
a.altitude = a.home_altitude
a.yaw = float(v[3])
a.ground_level = a.home_altitude
a.position.z = 0
a.wind = util.Wind(opts.wind)
a.set_yaw_degrees(a.yaw)

print("Starting at lat=%f lon=%f alt=%.1f heading=%.1f" % (
    a.home_latitude,
    a.home_longitude,
    a.home_altitude,
    a.yaw))

frame_time = 1.0/opts.rate

if opts.gimbal:
    from gimbal import Gimbal3Axis
    gimbal = Gimbal3Axis(a)
    print("Adding gimbal support")
else:
    gimbal = None

last_wall_time = time.time()

while True:
    frame_start = a.time_now
    sim_recv(m)

    m2 = m[:]

    a.update(m2)

    if gimbal is not None:
        gimbal.update()
    sim_send(m, a)

    now = time.time()
    if not opts.nowait and now < last_wall_time + frame_time:
        time.sleep(last_wall_time+frame_time - now)
    last_wall_time = time.time()

    a.time_advance(frame_time)
