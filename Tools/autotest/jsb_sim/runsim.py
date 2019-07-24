#!/usr/bin/env python
"""
 Run a jsbsim model as a child process.
"""
from __future__ import print_function
import atexit
import errno
import fdpexpect
import math
import os
import select
import signal
import socket
import struct
import sys
import time

import pexpect
from pymavlink import fgFDM

from .. pysim import util


class control_state(object):
    def __init__(self):
        self.aileron = 0
        self.elevator = 0
        self.throttle = 0
        self.rudder = 0
        self.ground_height = 0

sitl_state = control_state()


def interpret_address(addrstr):
    """Interpret a IP:port string."""
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)


def jsb_set(variable, value):
    """Set a JSBSim variable."""
    global jsb_console
    jsb_console.send('set %s %s\r\n' % (variable, value))


def setup_template(home):
    """Setup aircraft/Rascal/reset.xml ."""
    global opts
    v = home.split(',')
    if len(v) != 4:
        print("home should be lat,lng,alt,hdg - '%s'" % home)
        sys.exit(1)
    latitude = float(v[0])
    longitude = float(v[1])
    altitude = float(v[2])
    heading = float(v[3])
    sitl_state.ground_height = altitude
    template = os.path.join('aircraft', 'Rascal', 'reset_template.xml')
    reset = os.path.join('aircraft', 'Rascal', 'reset.xml')
    xml = open(template).read() % {'LATITUDE': str(latitude),
                                   'LONGITUDE': str(longitude),
                                   'HEADING': str(heading)}
    open(reset, mode='w').write(xml)
    print("Wrote %s" % reset)

    baseport = int(opts.simout.split(':')[1])

    template = os.path.join('jsb_sim', 'fgout_template.xml')
    out = os.path.join('jsb_sim', 'fgout.xml')
    xml = open(template).read() % {'FGOUTPORT': str(baseport+3)}
    open(out, mode='w').write(xml)
    print("Wrote %s" % out)

    template = os.path.join('jsb_sim', 'rascal_test_template.xml')
    out = os.path.join('jsb_sim', 'rascal_test.xml')
    xml = open(template).read() % {'JSBCONSOLEPORT': str(baseport+4)}
    open(out, mode='w').write(xml)
    print("Wrote %s" % out)


def process_sitl_input(buf):
    """Process control changes from SITL sim."""
    control = list(struct.unpack('<14H', buf))
    pwm = control[:11]
    (speed, direction, turbulance) = control[11:]

    global wind
    wind.speed = speed*0.01
    wind.direction = direction*0.01
    wind.turbulance = turbulance*0.01

    aileron = (pwm[0]-1500)/500.0
    elevator = (pwm[1]-1500)/500.0
    throttle = (pwm[2]-1000)/1000.0
    if opts.revthr:
        throttle = 1.0 - throttle
    rudder = (pwm[3]-1500)/500.0

    if opts.elevon:
        # fake an elevon plane
        ch1 = aileron
        ch2 = elevator
        aileron = (ch2-ch1)/2.0
        # the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0

    if opts.vtail:
        # fake an elevon plane
        ch1 = elevator
        ch2 = rudder
        # this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0
        rudder = (ch2+ch1)/2.0

    buf = ''
    if aileron != sitl_state.aileron:
        buf += 'set fcs/aileron-cmd-norm %s\n' % aileron
        sitl_state.aileron = aileron
    if elevator != sitl_state.elevator:
        buf += 'set fcs/elevator-cmd-norm %s\n' % elevator
        sitl_state.elevator = elevator
    if rudder != sitl_state.rudder:
        buf += 'set fcs/rudder-cmd-norm %s\n' % rudder
        sitl_state.rudder = rudder
    if throttle != sitl_state.throttle:
        buf += 'set fcs/throttle-cmd-norm %s\n' % throttle
        sitl_state.throttle = throttle
    buf += 'step\n'
    global jsb_console
    jsb_console.send(buf)


def update_wind(wind):
    """Update wind simulation."""
    (speed, direction) = wind.current()
    jsb_set('atmosphere/psiw-rad', math.radians(direction))
    jsb_set('atmosphere/wind-mag-fps', speed/0.3048)


def process_jsb_input(buf, simtime):
    """Process FG FDM input from JSBSim."""
    global fdm, fg_out, sim_out
    fdm.parse(buf)
    if fg_out:
        try:
            agl = fdm.get('agl', units='meters')
            fdm.set('altitude', agl+sitl_state.ground_height, units='meters')
            fdm.set('rpm', sitl_state.throttle*1000)
            fg_out.send(fdm.pack())
        except socket.error as e:
            if e.errno not in [errno.ECONNREFUSED]:
                raise

    timestamp = int(simtime*1.0e6)

    simbuf = struct.pack('<Q17dI',
                         timestamp,
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
        if e.errno not in [errno.ECONNREFUSED]:
            raise


################## main program ##################
from optparse import OptionParser
parser = OptionParser("runsim.py [options]")
parser.add_option("--simin",   help="SITL input (IP:port)",          default="127.0.0.1:5502")
parser.add_option("--simout",  help="SITL output (IP:port)",         default="127.0.0.1:5501")
parser.add_option("--fgout",   help="FG display output (IP:port)",   default="127.0.0.1:5503")
parser.add_option("--home",    type='string', help="home lat,lng,alt,hdg (required)")
parser.add_option("--script",  type='string', help='jsbsim model script', default='jsb_sim/rascal_test.xml')
parser.add_option("--options", type='string', help='jsbsim startup options')
parser.add_option("--elevon", action='store_true', default=False, help='assume elevon input')
parser.add_option("--revthr", action='store_true', default=False, help='reverse throttle')
parser.add_option("--vtail", action='store_true', default=False, help='assume vtail input')
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0')
parser.add_option("--rate", type='int', help="Simulation rate (Hz)", default=1000)
parser.add_option("--speedup", type='float', default=1.0, help="speedup from realtime")

(opts, args) = parser.parse_args()

for m in ['home', 'script']:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)
        parser.print_help()
        sys.exit(1)

os.chdir(util.reltopdir('Tools/autotest'))

# kill off child when we exit
atexit.register(util.pexpect_close_all)

setup_template(opts.home)

# start child
cmd = "JSBSim --realtime --suspend --nice --simulation-rate=%u --logdirectivefile=jsb_sim/fgout.xml --script=%s" % (opts.rate, opts.script)
if opts.options:
    cmd += ' %s' % opts.options

jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
jsb.delaybeforesend = 0
util.pexpect_autoclose(jsb)
i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                "Could not bind to socket for input"])
if i == 1:
    print("Failed to start JSBSim - is another copy running?")
    sys.exit(1)
jsb_out_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
jsb.expect("Creating UDP socket on port (\d+)")
jsb_in_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
jsb.expect("Successfully connected to socket for output")
jsb.expect("JSBSim Execution beginning")

# setup output to jsbsim
print("JSBSim console on %s" % str(jsb_out_address))
jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
jsb_out.connect(jsb_out_address)
jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
jsb_console.delaybeforesend = 0

# setup input from jsbsim
print("JSBSim FG FDM input on %s" % str(jsb_in_address))
jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
jsb_in.bind(jsb_in_address)
jsb_in.setblocking(0)

# socket addresses
sim_out_address = interpret_address(opts.simout)
sim_in_address = interpret_address(opts.simin)

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

jsb_console.send('info\n')
jsb_console.send('resume\n')
jsb.expect(["trim computation time", "Trim Results"])
time.sleep(1.5)
jsb_console.send('step\n')
jsb_console.logfile = None

print("Simulator ready to fly")


def main_loop():
    """Run main loop."""
    tnow = time.time()
    last_report = tnow
    last_sim_input = tnow
    last_wind_update = tnow
    frame_count = 0
    paused = False
    simstep = 1.0/opts.rate
    simtime = simstep
    frame_time = 1.0/opts.rate
    scaled_frame_time = frame_time/opts.speedup
    last_wall_time = time.time()
    achieved_rate = opts.speedup

    while True:
        new_frame = False
        rin = [jsb_in.fileno(), sim_in.fileno(), jsb_console.fileno(), jsb.fileno()]
        try:
            (rin, win, xin) = select.select(rin, [], [], 1.0)
        except select.error:
            util.check_parent()
            continue

        tnow = time.time()

        if jsb_in.fileno() in rin:
            buf = jsb_in.recv(fdm.packet_size())
            process_jsb_input(buf, simtime)
            frame_count += 1
            new_frame = True

        if sim_in.fileno() in rin:
            simbuf = sim_in.recv(28)
            process_sitl_input(simbuf)
            simtime += simstep
            last_sim_input = tnow

        # show any jsbsim console output
        if jsb_console.fileno() in rin:
            util.pexpect_drain(jsb_console)
        if jsb.fileno() in rin:
            util.pexpect_drain(jsb)

        # only simulate wind above 5 meters, to prevent crashes while
        # waiting for takeoff
        if tnow - last_wind_update > 0.1:
            update_wind(wind)
            last_wind_update = tnow

        if tnow - last_report > 3:
            print("FPS %u asl=%.1f agl=%.1f roll=%.1f pitch=%.1f a=(%.2f %.2f %.2f) AR=%.1f" % (
                frame_count / (time.time() - last_report),
                fdm.get('altitude', units='meters'),
                fdm.get('agl', units='meters'),
                fdm.get('phi', units='degrees'),
                fdm.get('theta', units='degrees'),
                fdm.get('A_X_pilot', units='mpss'),
                fdm.get('A_Y_pilot', units='mpss'),
                fdm.get('A_Z_pilot', units='mpss'),
                achieved_rate))

            frame_count = 0
            last_report = time.time()

        if new_frame:
            now = time.time()
            if now < last_wall_time + scaled_frame_time:
                dt = last_wall_time+scaled_frame_time - now
                time.sleep(last_wall_time+scaled_frame_time - now)
                now = time.time()

            if now > last_wall_time and now - last_wall_time < 0.1:
                rate = 1.0/(now - last_wall_time)
                achieved_rate = (0.98*achieved_rate) + (0.02*rate)
                if achieved_rate < opts.rate*opts.speedup:
                    scaled_frame_time *= 0.999
                else:
                    scaled_frame_time *= 1.001

            last_wall_time = now


def exit_handler():
    """Exit the sim."""
    print("running exit handler")
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    # JSBSim really doesn't like to die ...
    if getattr(jsb, 'pid', None) is not None:
        os.kill(jsb.pid, signal.SIGKILL)
    jsb_console.send('quit\n')
    jsb.close(force=True)
    util.pexpect_close_all()
    sys.exit(1)

signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)

try:
    main_loop()
except Exception as ex:
    print(ex)
    exit_handler()
    raise
