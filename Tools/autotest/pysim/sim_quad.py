#!/usr/bin/env python

from quadcopter import QuadCopter
import euclid, util, time, os, sys, math
import socket, struct
import select, fgFDM, errno

# find the mavlink.py module
for d in [ 'pymavlink',
           os.path.join(os.path.dirname(os.path.realpath(__file__)), '../pymavlink') ]:
    if os.path.exists(d):
        sys.path.insert(0, d)
import mavlink


def sim_send(m, a):
    '''send flight information to mavproxy and flightgear'''
    global fdm

    fdm.set('latitude', a.latitude, units='degrees')
    fdm.set('longitude', a.longitude, units='degrees')
    fdm.set('altitude', a.altitude, units='meters')
    fdm.set('phi', a.roll, units='degrees')
    fdm.set('theta', a.pitch, units='degrees')
    fdm.set('psi', a.yaw, units='degrees')
    fdm.set('phidot', a.roll_rate, units='dps')
    fdm.set('thetadot', a.pitch_rate, units='dps')
    fdm.set('psidot', a.yaw_rate, units='dps')
    fdm.set('vcas', math.sqrt(a.velocity.x*a.velocity.x + a.velocity.y*a.velocity.y), units='mps')
    fdm.set('v_north', a.velocity.x, units='mps')
    fdm.set('v_east', a.velocity.y, units='mps')
    fdm.set('num_engines', 4)
    for i in range(4):
        fdm.set('rpm', 1000*m[i], idx=i)
    try:
        fg_out.send(fdm.pack())
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise

    buf = struct.pack('<16dI',
                      a.latitude, a.longitude, a.altitude, a.yaw,
                      a.velocity.x, a.velocity.y,
                      a.accelerometer.x, a.accelerometer.y, a.accelerometer.z,
                      a.roll_rate, a.pitch_rate, a.yaw_rate,
                      a.roll, a.pitch, a.yaw,
                      math.sqrt(a.velocity.x*a.velocity.x + a.velocity.y*a.velocity.y),
                      0x4c56414e)
    try:
        sim_out.send(buf)
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise


def sim_recv(m):
    '''receive control information from SITL'''
    try:
        buf = sim_in.recv(22)
    except socket.error as e:
        if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            raise
        return
        
    if len(buf) != 22:
        return
    pwm = list(struct.unpack('<11H', buf))
    for i in range(4):
        m[i] = (pwm[i]-1000)/1000.0


def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

##################
# main program
from optparse import OptionParser
parser = OptionParser("sim_quad.py [options]")
parser.add_option("--fgout", dest="fgout",  help="flightgear output (IP:port)", default="127.0.0.1:5503")
parser.add_option("--simin",  dest="simin",   help="SIM input (IP:port)",       default="127.0.0.1:5502")
parser.add_option("--simout", dest="simout",  help="SIM output (IP:port)",      default="127.0.0.1:5501")
parser.add_option("--home", dest="home",  type='string', default=None, help="home lat,lng,alt,hdg (required)")
parser.add_option("--rate", dest="rate", type='int', help="SIM update rate", default=400)
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0')

(opts, args) = parser.parse_args()

for m in [ 'home' ]:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)
        parser.print_help()
        sys.exit(1)

parent_pid = os.getppid()

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
sim_in.setblocking(0)

# setup output to SITL
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(sim_out_address)
sim_out.setblocking(0)

# FG FDM object
fdm = fgFDM.fgFDM()

# create the quadcopter model
a = QuadCopter()

# motors initially off
m = [0, 0, 0, 0]

lastt = time.time()
frame_count = 0

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

print("Starting at lat=%f lon=%f alt=%.1f heading=%.1f" % (
    a.home_latitude,
    a.home_longitude,
    a.home_altitude,
    a.yaw))

frame_time = 1.0/opts.rate
sleep_overhead = 0

while True:
    frame_start = time.time()
    sim_recv(m)

    m2 = m[:]

    a.update(m2)
    sim_send(m, a)
    frame_count += 1
    t = time.time()
    if t - lastt > 1.0:
        print("%.2f fps zspeed=%.2f zaccel=%.2f h=%.1f a=%.1f yaw=%.1f yawrate=%.1f" % (
            frame_count/(t-lastt),
            a.velocity.z, a.accel.z, a.position.z, a.altitude,
            a.yaw, a.yaw_rate))
        lastt = t
        frame_count = 0
    frame_end = time.time()
    if frame_end - frame_start < frame_time:
        dt = frame_time - (frame_end - frame_start)
        dt -= sleep_overhead
        if dt > 0:
            time.sleep(dt)
        sleep_overhead = 0.99*sleep_overhead + 0.01*(time.time() - frame_end)
