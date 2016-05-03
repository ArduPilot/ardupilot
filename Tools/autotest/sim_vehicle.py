#!/usr/bin/env python

# framework to start a simulated vehicle and connect it to MAVProxy
# Peter Barker, April 2016
# based on sim_vehicle.sh by Andrew Tridgell, October 2011

import optparse
import sys
import psutil
import atexit
import os
import subprocess
import tempfile
import getpass
import time

class CompatError(Exception):
    '''a custom exception class to hold state if we encounter the parse error we are looking for'''
    def __init__(self,error, opts, rargs):
        Exception.__init__(self,error)
        self.opts = opts
        self.rargs = rargs

class CompatOptionParser(optparse.OptionParser):
    '''An option parser which emulates the behaviour of the old sim_vehicle.sh; if passed -C, the first argument not understood starts a list of arguments that are passed straight to mavproxy'''

    def __init__(self, *args, **kwargs):
        optparse.OptionParser.__init__(self, *args, **kwargs)

    def error(self, error):
        '''override default error handler called by optparse.OptionParser.parse_args when a parse error occurs; raise a detailed exception which can be caught'''
        if error.find("no such option") != -1:
            raise CompatError(error, self.values, self.rargs)

        optparse.OptionParser.error(self, error)

    def parse_args(self):
        '''wrap parse_args so we can catch the exception raised upon discovering the known parameter parsing error'''
        mavproxy_args = []
        try:
            opts, args = optparse.OptionParser.parse_args(self)
        except CompatError as e:
            if not e.opts.sim_vehicle_sh_compatible:
                print(e)
                print("Perhaps you want --sim_vehicle_sh_compatible?")
                sys.exit(1)
            if e.opts.mavproxy_args:
                print("--mavproxy-args not permitted in compat mode")
                sys.exit(1)

            args = []
            opts = e.opts
            mavproxy_args = []
            mavproxy_args.append(str(e)[16:]) # this trims "no such option" off
            mavproxy_args.extend(e.rargs)
            opts.ensure_value("mavproxy_args", " ".join(mavproxy_args))

        return opts, args

def kill_tasks():
    '''clean up stray processes by name.  This is a somewhat shotgun approach'''
    victim_names = set([
        'JSBSim',
        'lt-JSBSim',
        'ArduPlane.elf',
        'ArduCopter.elf',
        'APMrover2.elf',
        'AntennaTracker.elf',
        'JSBSIm.exe',
        'MAVProxy.exe',
        'runsim.py',
        'AntennaTracker.elf',
    ])
    for proc in psutil.process_iter():
        if proc.name() in victim_names:
            proc.kill()

# clean up processes at exit:
atexit.register(kill_tasks)

def check_jsbsim_version():
    '''assert that the JSBSim we will run is the one we expect to run'''
    jsbsim_version = subprocess.Popen(["JSBSim", "--version"], stdout=subprocess.PIPE).communicate()[0]
    try:
        jsbsim_version.index("ArduPilot")
    except ValueError:
        print('''
=========================================================
You need the latest ArduPilot version of JSBSim installed
and in your \$PATH

Please get it from git://github.com/tridge/jsbsim.git
See
  http://dev.ardupilot.org/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/ 
for more details
=========================================================
''')
        sys.exit(1)

def progress(text):
    '''display sim_vehicle progress text'''
    print("SIM_VEHICLE: " + text)

def find_autotest_dir():
    '''return path to autotest directory'''
    if os.path.exists("../Tools/autotest"):
        return "../Tools/autotest"

    # we are not running from one of the standard vehicle directories. Use
    # the location of the sim_vehicle script to find the path
    return os.path.dirname(os.path.realpath(__file__))


progress("Start")

# define and run parser
parser = CompatOptionParser("sim_vehicle.py", epilog='''
    eeprom.bin in the starting directory contains the parameters for your simulated vehicle. Always start from the same directory. It is recommended that you start in the main vehicle directory for the vehicle you are simulating, for example, start in the ArduPlane directory to simulate ArduPlane
''')
parser.add_option("-v", "--vehicle", type='string', default=None, help='vehicle type (ArduPlane, ArduCopter or APMrover2)')
parser.add_option("-I", "--instance", default=0, type='int', help='instance of simulator')
parser.add_option("-V", "--valgrind", action='store_true', default=False, help='enable valgrind for memory access checking (very slow!)')
parser.add_option("-N", "--no-rebuild", action='store_true', default=False, help="don't rebuild before starting ardupilot")
parser.add_option("-H", "--hil", action='store_true', default=False, help="start HIL")
parser.add_option("-T", "--tracker", action='store_true', default=False, help="start an antenna tracker instance")
parser.add_option("-A", "--sitl-instance-args", type='string', default=None, help='pass arguments to SITL instance')
parser.add_option("-R", "--reverse-throttle", action='store_true', default=False, help="reverse throttle in plane")
parser.add_option("-G", "--gdb", action='store_true', default=False, help="use gdb for debugging ardupilot")
parser.add_option("-g", "--gdb-stopped", action='store_true', default=False, help="use gdb for debugging ardupilot (no auto-start)")
parser.add_option("-D", "--debug", action='store_true', default=False, help="build with debugging")
parser.add_option("-d", "--delay-start", default=0, type='float', help='delays the start of mavproxy by the number of seconds')
parser.add_option("-B", "--breakpoint", type='string', default=None, help='add a breakpoint at given location in debugger')
parser.add_option("-M", "--mavlink-gimbal", action='store_true', default=False, help='enable MAVLink gimbal')
parser.add_option("-L", "--location", type='string', default='CMAC', help='select start location from Tools/autotest/locations.txt')
parser.add_option("-l", "--custom-location", type='string', default=None, help='set custom start location')
parser.add_option("-f", "--frame", type='string', default=None, help='''set aircraft frame type
                     for copters can choose +, X, quad or octa
                     for planes can choose elevon or vtail''')
parser.add_option("-S", "--speedup", default=1, type='int', help='set simulation speedup (1 for wall clock time)')
parser.add_option("-t", "--tracker-location", default='CMAC_PILOTSBOX', type='string', help='set antenna tracker start location')
parser.add_option("-c", "--clean", action='store_true', default=False, help='do a make clean before building')
parser.add_option("-j", "--jobs", default=1, type='int', help='number of processors to use during build (default 1)')
parser.add_option("-w", "--wipe-eeprom", action='store_true', default=False, help='wipe EEPROM and reload parameters')
parser.add_option("-b", "--build-target", default=None, type='string', help='override SITL build target (will be set to sitl if not specified)')
parser.add_option("-m", "--mavproxy-args", default=None, type='string', help='additional arguments to pass to mavproxy.py')
parser.add_option("-C", "--sim_vehicle_sh_compatible", action='store_true', default=False, help='be compatible with the way sim_vehicle.sh works; make this the first option')

opts, args = parser.parse_args()

# validate parameters
if opts.hil:
    opts.no_rebuild = True
    if opts.valgrind:
        print("May not use valgrind with hil")
        sys.exit(1)
    if opts.gdb or opts.gdb_stopped:
        print("May not use gdb with hil")
        sys.exit(1)

if opts.valgrind and (opts.gdb or opts.gdb_stopped):
    print("May not use valgrind with gdb")
    sys.exit(1)

# magically determine vehicle type (if required):
if opts.vehicle is None:
    cwd = os.getcwd()
    opts.vehicle = os.path.basename(cwd)

# determine a frame type if not specified:
default_frame_for_vehicle = {
    "APMrover2": "rover",
    "ArduPlane": "jsbsim",
    "ArduCopter": "quad",
    "AntennaTracker": "tracker"
}

# determine frame options (e.g. build type might be "sitl")
if opts.frame is None:
    opts.frame = default_frame_for_vehicle[opts.vehicle]

# setup ports for this instance
mavlink_port = "tcp:127.0.0.1:" + str(5760 + 10*opts.instance)
simout_port = "127.0.0.1:" + str(5501 + 10*opts.instance)

''' understood entries:
build_target: option passed to make to create binaries.  Usually sitl, and "-debug" may be appended if -D is passed to sim_vehicle.py
default_params_filename: filename of default parameters file.  Taken to be relative to autotest dir.
extra_mavlink_cmds: extra parameters that will be passed to mavproxy
'''
_options_for_frame = {
    "+": {
        "default_params_filename": "copter_params.parm"
    },
    "quad": {
        "model": "+",
        "default_params_filename": "copter_params.parm"
    },
    "X": {
        # this param set FRAME doesn't actually work because mavproxy
        # won't set a parameter unless it knows of it, and the param fetch happens asynchronously
        "extra_mavlink_cmds": "param fetch frame; param set FRAME 1;",
        "default_params_filename": "copter_params.parm"
    },
    "heli-dual": {
        "build_target": "sitl-heli-dual",
    },
    "heli-compound": {
        "build_target": "sitl-heli-compound",
    },
    "IrisRos": {
        "default_params_filename": "copter_params.parm"
    },
    "Gazebo": {
        "default_params_filename": "copter_params.parm"
    },

    "octa": {
	"build_target": "sitl-octa",
        "default_params_filename": "copter_params.parm",
    },
    "tri": {
	"build_target": "sitl-tri",
        "default_params_filename": "tri_params.parm",
    },
    "y6": {
	"build_target": "sitl-y6",
        "default_params_filename": "y6_params.parm",
    },
    "firefly": {
        "default_params_filename": "firefly.parm",
    },
    "heli": {
	"build_target": "sitl-heli",
        "default_params_filename": "Helicopter.parm",
    },
    "last_letter": {
    },
    "CRRCSim": {
    },
    "jsbsim": {
        "default_params_filename": "ArduPlane.parm",
    },
    "quadplane": {
        "default_params_filename": "quadplane.parm",
    },
    "plane-elevon": {
        "default_params_filename": "plane-elevons.parm",
    },
    "plane-vtail": {
        "default_params_filename": "plane-vtail.parm",
    },
    "plane": {
        "default_params_filename": "plane.parm",
    },
}

def options_for_frame(frame, opts):
    '''return informatiom about how to sitl for frame e.g. build-type==sitl'''
    if frame in _options_for_frame:
        ret = _options_for_frame[frame]
    else:
        for p in [ "octa", "tri", "y6", "firefly", "heli", "last_letter", "jsbsim", "quadplane", "plane-elevon", "plane-vtail", "plane" ]:
            if frame.startswith(p):
                ret = _options_for_frame[p]
                break
    if ret is None:
        if frame.endswith("-heli"):
            ret = _options_for_frame["heli"]
    if ret is None:
        ret = {}

    if not ret.has_key("model"):
        ret["model"] = frame
    if not ret.has_key("build_target"):
        ret["build_target"] = "sitl"

    if opts.build_target is not None:
        ret["build_target"] = opts.build_target
    elif opts.debug:
        ret["build_target"] += "-debug"

    return ret

def do_build(vehicledir, opts, build_target):
    '''build build_target (e.g. sitl) in directory vehicledir'''
    old_dir = os.getcwd()

    os.chdir(vehicledir)

    if opts.clean:
        progress("Building clean")
        subprocess.Popen(["make", "clean"])

    progress("Building %s" % (build_target))
    p = subprocess.Popen(["make", "-j"+str(opts.jobs), build_target])
    pid, sts = os.waitpid(p.pid,0)
    if sts != 0:
        progress("Build failed; cleaning and rebuilding")
        subprocess.Popen(["make", "clean"])
        p = subprocess.Popen(["make", "-j"+str(opts.jobs), build_target])
        pid, sts = os.waitpid(p.pid,0)
        if sts != 0:
            progress("Build failed")
            sys.exit(1)

    os.chdir(old_dir)

def find_location_by_name(autotest, locname):
    '''search locations.txt for locname, return GPS coords'''
    locations_filepath = os.path.join(autotest, "locations.txt")
    for line in open(locations_filepath,'r'):
        line = line.rstrip("\n")
        (name,loc) = line.split("=")
        if name == locname:
            return loc
    print("Failed to find location (%s)" % (opts.location))
    sys.exit(1)

def run_in_terminal_window(autotest, name, cmd):
    '''execute the run_in_terminal_window.sh command for cmd'''
    runme = [os.path.join(autotest, "run_in_terminal_window.sh"), name]
    runme.extend(cmd)
    progress("%s" % (str(runme),))
    p = subprocess.Popen(runme) # bg this explicitly?!

tracker_uarta = None # blemish

def start_antenna_tracker(autotest, opts):
    '''compile and run the AntennaTracker, add tracker to mavproxy'''
    global tracker_uarta
    progress("Preparing antenna tracker")
    tracker_home = find_location_by_name(find_autotest_dir(), opts.tracker_location)
    vehicledir = os.path.join(autotest, "../../" + "AntennaTracker")
    do_build(vehicledir, opts, "sitl-debug")
    tracker_instance = 1
    os.chdir(vehicledir)
    tracker_uarta = "tcp:127.0.0.1:" + str(5760+10*tracker_instance)
    exe = os.path.join(vehicledir, "AntennaTracker.elf")
    run_in_terminal_window(autotest, "AntennaTracker", ["nice", exe, "-I" + str(tracker_instance), "--model=tracker", "--home="+tracker_home])

def start_vehicle(vehicledir, autotest, opts, stuff, loc):
    '''run the ArduPilot binary found in vehicledir'''
    progress("Starting %s" % (opts.vehicle,))
    cmd_name = opts.vehicle
    cmd = []
    if opts.valgrind:
        progress("Using valgrind")
        cmd_name += " (valgrind)"
        cmd.append("valgrind")
    if opts.gdb:
        progress("Using gdb")
        cmd_name += " (gdb)"
        cmd.append("gdb")
        gdb_commands_file = tempfile.NamedTemporaryFile(delete=False)
        atexit.register(os.unlink, gdb_commands_file.name)

        if opts.breakpoint:
            gdb_commands_file.write("b %s\n" % (opts.breakpoint,))
        gdb_commands_file.write("r\n")
        gdb_commands_file.close()
        cmd.extend(["-x", gdb_commands_file.name])
        cmd.append("--args")

    cmd.append(os.path.join(vehicledir, opts.vehicle+".elf"))
    cmd.append("-S")
    cmd.append("-I"+str(opts.instance))
    cmd.extend(["--home", loc])
    if opts.wipe_eeprom:
        cmd.append("-w")
    cmd.extend(["--model", stuff["model"]])
    cmd.extend(["--speedup", str(opts.speedup)])
    if opts.sitl_instance_args:
        cmd.extend(opts.sitl_instance_args.split(" ")) # this could be a lot better..
    if opts.mavlink_gimbal:
        cmd.append("--gimbal")
    if stuff.has_key("default_params_filename"):
        path = os.path.join(autotest, stuff["default_params_filename"])
        progress("Using defaults from (%s)" % (path,))
        cmd.extend(["--defaults", path])

    run_in_terminal_window(autotest, cmd_name, cmd)

def start_mavproxy(opts, stuff):
    '''run mavproxy'''
    # FIXME: would be nice to e.g. "mavproxy.mavproxy(....).run" rather than shelling out
    progress("Starting MAVProxy")

    extra_cmd = ""
    cmd = []
    cygstart = "/usr/bin/cygstart"
    if os.path.exists(cygstart):
        cmd.append(cygstart)
        cmd.append("-w")
        cmd.append("/cygdrive/c/Program Files (x86)/MAVProxy/mavproxy.exe")
    else:
        cmd.append("mavproxy.py")

    if opts.hil:
        cmd.extend(["--load-module", "HIL"])
    else:
        cmd.extend(["--master", mavlink_port, "--sitl", simout_port])
    # If running inside of a vagrant guest, then we probably want to forward our mavlink out to the containing host OS
    if getpass.getuser() == "vagrant":
        cmd.extend(["--out", "10.0.2.2:14550"])
    for port in [14550, 14551]:
        cmd.extend(["--out", "127.0.0.1:"+str(port)])

    if opts.tracker:
        cmd.extend(["--load-module", "tracker"])
        global tracker_uarta
        # tracker_uarta is set when we start the tracker...
        extra_cmd += "module load map; tracker set port %s; tracker start; tracker arm;" % (tracker_uarta,)

    if opts.mavlink_gimbal:
        cmd.extend(["--load-module", "gimbal"])

    if stuff.has_key("extra_mavlink_cmds"):
        extra_cmd += " " + stuff["extra_mavlink_cmds"]

    if opts.mavproxy_args:
        cmd.extend(opts.mavproxy_args.split(" ")) # this could be a lot better..

    if len(extra_cmd):
        cmd.extend(['--cmd', extra_cmd])

    progress("%s" % (cmd,))
    p = subprocess.Popen(cmd)
    pid, sts = os.waitpid(p.pid,0)
    progress("MAVProxy exitted")

frame_options = options_for_frame(opts.frame, opts)

if frame_options["model"] == "jsbsim":
    check_jsbsim_version()

vehicledir = os.path.join(find_autotest_dir(), "../../" + opts.vehicle)
if not os.path.exists(vehicledir):
    print("vehicle directory (%s) does not exist" % (vehicledir,))
    sys.exit(1)

os.environ['AUTOTEST'] = find_autotest_dir() # should we only drop this in subprocess env?

if not opts.hil:
    if opts.instance == 0:
        kill_tasks()

if opts.tracker:
    start_antenna_tracker(find_autotest_dir(), opts)

# rebuild time:
if not opts.no_rebuild: # i.e. we should rebuild
    do_build(vehicledir, opts, frame_options["build_target"])

if opts.custom_location:
    loc = opts.custom_location
    progress("Starting up at %s" % (loc,))
else:
    loc = find_location_by_name(find_autotest_dir(), opts.location)
    progress("Starting up at %s (%s)" % (loc, opts.location))

if opts.hil:
    # (unlikely)
    run_in_terminal_window(find_autotest_dir(), "JSBSim", [ os.path.join(find_autotest_dir(), "jsb_sim/runsim.py"), "--home", loc, "--speedup=" + str(opts.speedup)])
else:
    start_vehicle(vehicledir, find_autotest_dir(), opts, frame_options, loc)

if opts.delay_start:
    progress("Sleeping for %f seconds" % (opts.delay_start,))
    time.sleep(float(opts.delay_start))

start_mavproxy(opts, frame_options)

sys.exit(0)
