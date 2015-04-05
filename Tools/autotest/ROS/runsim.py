#!/usr/bin/env python
# run a ROS simulator as a child process

import sys, os, pexpect, socket
import math, time, select, struct, signal, errno

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'pysim'))

import util, atexit
from pymavlink import fgFDM

class control_state(object):
    def __init__(self):
        self.aileron = 0
        self.elevator = 0
        self.throttle = 0
        self.rudder = 0
        self.ground_height = 0

sitl_state = control_state()


def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

def process_sitl_input(buf):
    '''process control changes from SITL sim'''

    global ros_out

    control = list(struct.unpack('<14H', buf))
    pwm = control[:11]
    (speed, direction, turbulance) = control[11:]

    global wind
    wind.speed      = speed*0.01
    wind.direction  = direction*0.01
    wind.turbulance = turbulance*0.01

    aileron  = (pwm[0]-1500)/500.0
    elevator = (pwm[1]-1500)/500.0
    throttle = (pwm[2]-1000)/1000.0
    if opts.revthr:
        throttle = 1.0 - throttle
    rudder   = (pwm[3]-1500)/500.0

    if opts.elevon:
        # fake an elevon plane
        ch1 = aileron
        ch2 = elevator
        aileron  = (ch2-ch1)/2.0
        # the minus does away with the need for RC2_REV=-1
        elevator = -(ch2+ch1)/2.0

    if opts.vtail:
        # fake an elevon plane
        ch1 = elevator
        ch2 = rudder
        # this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0
        rudder   = (ch2+ch1)/2.0

    if aileron != sitl_state.aileron:
        fdm_ctrls.set('left_aileron', pwm[0])
        sitl_state.aileron = aileron
    if elevator != sitl_state.elevator:
        fdm_ctrls.set('elevator', pwm[1])
        sitl_state.elevator = elevator
    if rudder != sitl_state.rudder:
        fdm_ctrls.set('rudder', pwm[3])
        sitl_state.rudder = rudder
    if throttle != sitl_state.throttle:
        fdm_ctrls.set('rpm', pwm[2])
        sitl_state.throttle = throttle

    try:
        ros_out.send(fdm_ctrls.pack())
    except socket.error as e:
        if e.errno not in [ errno.ECONNREFUSED ]:
            raise

def process_ros_input(buf,frame_count):
    '''process FG FDM input from ROS Simulator'''
    global fdm, fg_out, sim_out
    FG_FDM_FPS = 30

    fdm.parse(buf)
    if (fg_out and ((frame_count % FG_FDM_FPS) == 0)) :
        try:
            agl = fdm.get('agl', units='meters')
            fdm.set('altitude', agl+sitl_state.ground_height, units='meters')
            fdm.set('rpm', sitl_state.throttle*1000)
            fg_out.send(fdm.pack())
        except socket.error as e:
            if e.errno not in [ errno.ECONNREFUSED ]:
                raise

    simbuf = struct.pack('<17dI',
                         fdm.get('latitude', units='degrees'),
                         fdm.get('longitude', units='degrees'),
                         fdm.get('altitude', units='meters'),
                         fdm.get('psi', units='degrees'),
                         fdm.get('v_north', units='mps'),
                         fdm.get('v_east', units='mps'),
                         fdm.get('v_down', units='mps'),
                         fdm.get('A_X_pilot', units='mpss'),
                         fdm.get('A_Y_pilot', units='mpss'),
                         fdm.get('A_Z_pilot', units='mpss'),
                         fdm.get('phidot', units='dps'),
                         fdm.get('thetadot', units='dps'),
                         fdm.get('psidot', units='dps'),
                         fdm.get('phi', units='degrees'),
                         fdm.get('theta', units='degrees'),
                         fdm.get('psi', units='degrees'),
                         fdm.get('vcas', units='mps'),
                         0x4c56414f)
    try:
        sim_out.send(simbuf)
    except socket.error as e:
        if e.errno not in [ errno.ECONNREFUSED ]:
            raise



##################
# main program
from optparse import OptionParser
parser = OptionParser("runsim.py [options]")
parser.add_option("--simin",   help="SITL input (IP:port)",          default="127.0.0.1:5502")
parser.add_option("--simout",  help="SITL output (IP:port)",         default="127.0.0.1:5501")
parser.add_option("--fgout",   help="FG display output (IP:port)",   default="127.0.0.1:5503")
parser.add_option("--home",    type='string', help="home lat,lng,alt,hdg") # Not implemented yet
parser.add_option("--script",  type='string', help='jsbsim model script', default='jsbsim/rascal_test.xml') # Not implemented yet
parser.add_option("--options", type='string', help='ROS startup options')
parser.add_option("--elevon", action='store_true', default=False, help='assume elevon input')
parser.add_option("--revthr", action='store_true', default=False, help='reverse throttle')
parser.add_option("--vtail", action='store_true', default=False, help='assume vtail input')
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0') # Not implemented yet

(opts, args) = parser.parse_args()

os.chdir(util.reltopdir('Tools/autotest'))

# kill off child when we exit
atexit.register(util.pexpect_close_all)

# start child
cmd = "roslaunch last_letter launcher.launch ArduPlane:=true"
if opts.options:
    cmd += ' %s' % opts.options

ros = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
ros.delaybeforesend = 0
util.pexpect_autoclose(ros)

ros_out_address = interpret_address("127.0.0.1:5505")
ros_in_address = interpret_address("127.0.0.1:5504")

# setup output to ROS
print("ROS listens for FG-FDM packets at %s" % str(ros_out_address))
ros_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ros_out.connect(ros_out_address)

# setup input from ROS
print("ROS sends FG-FDM packetes for the SITL at %s" % str(ros_in_address))
ros_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ros_in.bind(ros_in_address)
ros_in.setblocking(0)

# socket addresses
sim_out_address = interpret_address(opts.simout)
sim_in_address  = interpret_address(opts.simin)

# setup input from SITL sim
sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_in.bind(sim_in_address)
sim_in.setblocking(0)

# setup output to SITL sim
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(interpret_address(opts.simout))
sim_out.setblocking(0)

# setup possible output to FlightGear for display
fg_out = None
if opts.fgout:
    fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fg_out.connect(interpret_address(opts.fgout))

# setup wind generator
wind = util.Wind(opts.wind)

fdm = fgFDM.fgFDM()
fdm_ctrls = fgFDM.fgFDM() # Setup another fdm object to send ctrls to ROS

time.sleep(1.5)

print("Simulator ready to fly")

def main_loop():
    '''run main loop'''
    tnow = time.time()
    last_report = tnow
    last_sim_input = tnow
    last_wind_update = tnow
    frame_count = 0
    paused = False

    while True:
        rin = [ros_in.fileno(), sim_in.fileno()]

        try:
            (rin, win, xin) = select.select(rin, [], [], 1.0)
        except select.error:
            util.check_parent()
            continue

        tnow = time.time()

        if ros_in.fileno() in rin:
            buf = ros_in.recv(fdm.packet_size())
            process_ros_input(buf,frame_count)
            frame_count += 1

        if sim_in.fileno() in rin:
            simbuf = sim_in.recv(28)
            process_sitl_input(simbuf)
            last_sim_input = tnow

        if tnow - last_report > 3:
            print("FPS %u asl=%.1f agl=%.1f roll=%.1f pitch=%.1f a=(%.2f %.2f %.2f)" % (
                frame_count / (time.time() - last_report),
                fdm.get('altitude', units='meters'),
                fdm.get('agl', units='meters'),
                fdm.get('phi', units='degrees'),
                fdm.get('theta', units='degrees'),
                fdm.get('A_X_pilot', units='mpss'),
                fdm.get('A_Y_pilot', units='mpss'),
                fdm.get('A_Z_pilot', units='mpss')))

            frame_count = 0
            last_report = time.time()

def exit_handler():
    '''exit the sim'''
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    sys.exit(1)

signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)

try:
    main_loop()
except:
    exit_handler()
    raise
