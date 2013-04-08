#!/usr/bin/env python
'''
simple rover simulator
'''

from rover import Rover
import util, time, os, sys, math
import socket, struct
import select, errno

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'mavlink', 'pymavlink'))

def sim_send(a):
    '''send flight information to mavproxy'''
    from math import degrees

    earth_rates = util.BodyRatesToEarthRates(a.dcm, a.gyro)
    (roll, pitch, yaw) = a.dcm.to_euler()

    buf = struct.pack('<17dI',
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


def sim_recv(state):
    '''receive control information from SITL'''
    try:
        buf = sim_in.recv(28)
    except socket.error as e:
        if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            raise
        return
        
    if len(buf) != 28:
        print('len=%u' % len(buf))
        return
    control = list(struct.unpack('<14H', buf))
    pwm = control[0:11]

    # map steering and throttle to -1/1
    state.steering = (pwm[0]-1500)/500.0
    state.throttle = (pwm[2]-1500)/500.0

#    print("steering=%f throttle=%f pwm=%s" % (state.steering, state.throttle, str(pwm)))
    


def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

class ControlState:
    def __init__(self):
        # steering from -1 to 1, where -1 is left, 1 is right
        self.steering = 0
        # throttle from -1 to 1, where -1 is full reverse, 1 is full forward
        self.throttle = 0

##################
# main program
from optparse import OptionParser
parser = OptionParser("sim_rover.py [options]")
parser.add_option("--simin",  dest="simin",   help="SIM input (IP:port)",       default="127.0.0.1:5502")
parser.add_option("--simout", dest="simout",  help="SIM output (IP:port)",      default="127.0.0.1:5501")
parser.add_option("--home", dest="home",  type='string', default=None, help="home lat,lng,alt,hdg (required)")
parser.add_option("--rate", dest="rate", type='int', help="SIM update rate", default=100)
parser.add_option("--skid-steering", action='store_true', default=False, help="Use skid steering")

(opts, args) = parser.parse_args()

for m in [ 'home' ]:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)
        parser.print_help()
        sys.exit(1)

# UDP socket addresses
sim_out_address = interpret_address(opts.simout)
sim_in_address  = interpret_address(opts.simin)

# setup input from SITL
sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_in.bind(sim_in_address)
sim_in.setblocking(0)

# setup output to SITL
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(sim_out_address)
sim_out.setblocking(0)

# create the quadcopter model
a = Rover(skid_steering=opts.skid_steering)

# initial controls state
state = ControlState()


# parse home
v = opts.home.split(',')
if len(v) != 4:
    print("home should be lat,lng,alt,hdg")
    sys.exit(1)
a.home_latitude  = float(v[0])
a.home_longitude = float(v[1])
a.home_altitude  = float(v[2])
a.altitude       = a.home_altitude
a.yaw            = float(v[3])
a.latitude = a.home_latitude
a.longitude = a.home_longitude

print("Starting at lat=%f lon=%f alt=%f heading=%.1f" % (
    a.home_latitude,
    a.home_longitude,
    a.altitude,
    a.yaw))

frame_time = 1.0/opts.rate
sleep_overhead = 0

while True:
    frame_start = time.time()
    sim_recv(state)
    a.update(state)
    sim_send(a)
    t = time.time()
    frame_end = time.time()
    if frame_end - frame_start < frame_time:
        dt = frame_time - (frame_end - frame_start)
        dt -= sleep_overhead
        if dt > 0:
            time.sleep(dt)
        sleep_overhead = 0.99*sleep_overhead + 0.01*(time.time() - frame_end)
