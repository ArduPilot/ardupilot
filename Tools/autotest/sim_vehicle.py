#!/usr/bin/env python

"""
Framework to start a simulated vehicle and connect it to MAVProxy.

Peter Barker, April 2016
based on sim_vehicle.sh by Andrew Tridgell, October 2011
"""
from __future__ import print_function

import atexit
import errno
import optparse
import os
import os.path
import re
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import shlex

from pysim import vehicleinfo

# List of open terminal windows for macosx
windowID = []


class CompatError(Exception):
    """A custom exception class to hold state if we encounter the parse
    error we are looking for"""
    def __init__(self, error, opts, rargs):
        Exception.__init__(self, error)
        self.opts = opts
        self.rargs = rargs


class CompatOptionParser(optparse.OptionParser):
    """An option parser which emulates the behaviour of the old
    sim_vehicle.sh; if passed -C, the first argument not understood starts
    a list of arguments that are passed straight to mavproxy
    """

    class CustomFormatter(optparse.IndentedHelpFormatter):
        def __init__(self, *args, **kwargs):
            optparse.IndentedHelpFormatter.__init__(self, *args, **kwargs)

        # taken and modified from from optparse.py's format_option
        def format_option_preserve_nl(self, option):
            # The help for each option consists of two parts:
            #   * the opt strings and metavars
            #     eg. ("-x", or "-fFILENAME, --file=FILENAME")
            #   * the user-supplied help string
            #     eg. ("turn on expert mode", "read data from FILENAME")
            #
            # If possible, we write both of these on the same line:
            #   -x      turn on expert mode
            #
            # But if the opt string list is too long, we put the help
            # string on a second line, indented to the same column it would
            # start in if it fit on the first line.
            #   -fFILENAME, --file=FILENAME
            #           read data from FILENAME
            result = []
            opts = self.option_strings[option]
            opt_width = self.help_position - self.current_indent - 2
            if len(opts) > opt_width:
                opts = "%*s%s\n" % (self.current_indent, "", opts)
            else:                       # start help on same line as opts
                opts = "%*s%-*s  " % (self.current_indent, "", opt_width, opts)
            result.append(opts)
            if option.help:
                help_text = self.expand_default(option)
                tw = textwrap.TextWrapper(replace_whitespace=False,
                                          initial_indent="",
                                          subsequent_indent="    ",
                                          width=self.help_width)

                for line in help_text.split("\n"):
                    help_lines = tw.wrap(line)
                    for wline in help_lines:
                        result.extend(["%*s%s\n" % (self.help_position,
                                                    "",
                                                    wline)])
            elif opts[-1] != "\n":
                result.append("\n")
            return "".join(result)

        def format_option(self, option):
            if str(option).find('frame') != -1:
                return self.format_option_preserve_nl(option)
            return optparse.IndentedHelpFormatter.format_option(self, option)

    def __init__(self, *args, **kwargs):
        formatter = CompatOptionParser.CustomFormatter()
        optparse.OptionParser.__init__(self,
                                       *args,
                                       formatter=formatter,
                                       **kwargs)

    def error(self, error):
        """Override default error handler called by
        optparse.OptionParser.parse_args when a parse error occurs;
        raise a detailed exception which can be caught
        """
        if error.find("no such option") != -1:
            raise CompatError(error, self.values, self.rargs)

        optparse.OptionParser.error(self, error)

    def parse_args(self, args=None, values=None):
        '''Wrap parse_args so we can catch the exception raised upon
        discovering the known parameter parsing error
        '''

        try:
            opts, args = optparse.OptionParser.parse_args(self)
        except CompatError as e:
            if not e.opts.sim_vehicle_sh_compatible:
                print(e)
                print("Perhaps you want --sim_vehicle_sh_compatible (-C)?")
                sys.exit(1)
            if e.opts.mavproxy_args:
                print("--mavproxy-args not permitted in compat mode")
                sys.exit(1)

            args = []
            opts = e.opts
            mavproxy_args = [str(e)[16:]]  # this trims "no such option" off
            mavproxy_args.extend(e.rargs)
            opts.ensure_value("mavproxy_args", " ".join(mavproxy_args))

        return opts, args


def cygwin_pidof(proc_name):
    """ Thanks to kata198 for this:
    https://github.com/kata198/cygwin-ps-misc/blob/master/pidof
    """
    pipe = subprocess.Popen("ps -ea | grep " + proc_name,
                            shell=True,
                            stdout=subprocess.PIPE)
    output_lines = pipe.stdout.read().replace("\r", "").split("\n")
    ret = pipe.wait()
    pids = []
    if ret != 0:
        # No results
        return []
    for line in output_lines:
        if not line:
            continue
        line_split = [item for item in line.split(' ') if item]
        cmd = line_split[-1].split('/')[-1]
        if cmd == proc_name:
            try:
                pid = int(line_split[0].strip())
            except:
                pid = int(line_split[1].strip())
            if pid not in pids:
                pids.append(pid)
    return pids


def under_cygwin():
    """Return if Cygwin binary exist"""
    return os.path.exists("/usr/bin/cygstart")


def under_macos():
    return sys.platform == 'darwin'


def kill_tasks_cygwin(victims):
    """Shell out to ps -ea to find processes to kill"""
    for victim in list(victims):
        pids = cygwin_pidof(victim)
#        progress("pids for (%s): %s" %
#                 (victim,",".join([ str(p) for p in pids])))
        for apid in pids:
            os.kill(apid, signal.SIGKILL)


def kill_tasks_macos():
    for window in windowID:
        cmd = ("osascript -e \'tell application \"Terminal\" to close "
               "(window(get index of window id %s))\'" % window)
        os.system(cmd)


def kill_tasks_psutil(victims):
    """Use the psutil module to kill tasks by name.  Sadly, this module is
    not available on Windows, but when it is we should be able to *just*
    use this routine"""
    import psutil
    for proc in psutil.process_iter():
        if proc.status == psutil.STATUS_ZOMBIE:
            continue
        if proc.name in victims:
            proc.kill()


def kill_tasks_pkill(victims):
    """Shell out to pkill(1) to kill processed by name"""
    for victim in victims:  # pkill takes a single pattern, so iterate
        cmd = ["pkill", victim[:15]]  # pkill matches only first 15 characters
        run_cmd_blocking("pkill", cmd, quiet=True)


class BobException(Exception):
    """Handle Bob's Exceptions"""
    pass


def kill_tasks():
    """Clean up stray processes by name.  This is a shotgun approach"""
    progress("Killing tasks")
    try:
        victim_names = {
            'JSBSim',
            'lt-JSBSim',
            'ArduPlane.elf',
            'ArduCopter.elf',
            'ArduSub.elf',
            'APMrover2.elf',
            'AntennaTracker.elf',
            'JSBSIm.exe',
            'MAVProxy.exe',
            'runsim.py',
            'AntennaTracker.elf',
        }
        for vehicle in vinfo.options:
            for frame in vinfo.options[vehicle]["frames"]:
                frame_info = vinfo.options[vehicle]["frames"][frame]
                if "waf_target" not in frame_info:
                    continue
                exe_name = os.path.basename(frame_info["waf_target"])
                victim_names.add(exe_name)

        if under_cygwin():
            return kill_tasks_cygwin(victim_names)
        if under_macos() and os.environ.get('DISPLAY'):
            # use special macos kill routine if Display is on
            return kill_tasks_macos()

        try:
            kill_tasks_psutil(victim_names)
        except ImportError:
            kill_tasks_pkill(victim_names)
    except Exception as e:
        progress("kill_tasks failed: {}".format(str(e)))


def progress(text):
    """Display sim_vehicle progress text"""
    print("SIM_VEHICLE: " + text)


def find_autotest_dir():
    """Return path to autotest directory"""
    return os.path.dirname(os.path.realpath(__file__))


def find_root_dir():
    """Return path to root directory"""
    return os.path.realpath(os.path.join(find_autotest_dir(), '../..'))


def wait_unlimited():
    """Wait until signal received"""
    while True:
        time.sleep(600)


vinfo = vehicleinfo.VehicleInfo()


def do_build_waf(opts, frame_options):
    """Build sitl using waf"""
    progress("WAF build")

    old_dir = os.getcwd()
    root_dir = find_root_dir()
    os.chdir(root_dir)

    waf_light = os.path.join(root_dir, "modules/waf/waf-light")

    cmd_configure = [waf_light, "configure", "--board", "sitl"]
    if opts.debug:
        cmd_configure.append("--debug")

    if opts.OSD:
        cmd_configure.append("--enable-sfml")

    pieces = [shlex.split(x) for x in opts.waf_configure_args]
    for piece in pieces:
        cmd_configure.extend(piece)

    run_cmd_blocking("Configure waf", cmd_configure, check=True)

    if opts.clean:
        run_cmd_blocking("Building clean", [waf_light, "clean"])

    cmd_build = [waf_light, "build", "--target", frame_options["waf_target"]]
    if opts.jobs is not None:
        cmd_build += ['-j', str(opts.jobs)]
    pieces = [shlex.split(x) for x in opts.waf_build_args]
    for piece in pieces:
        cmd_build.extend(piece)

    _, sts = run_cmd_blocking("Building", cmd_build)

    if sts != 0:  # build failed
        if opts.rebuild_on_failure:
            progress("Build failed; cleaning and rebuilding")
            run_cmd_blocking("Building clean", [waf_light, "clean"])

            _, sts = run_cmd_blocking("Building", cmd_build)
            if sts != 0:
                progress("Build failed")
                sys.exit(1)
        else:
            progress("Build failed")
            sys.exit(1)

    os.chdir(old_dir)


def do_build_parameters(vehicle):
    # build succeeded
    # now build parameters
    progress("Building fresh parameter descriptions")
    param_parse_path = os.path.join(
        find_root_dir(), "Tools/autotest/param_metadata/param_parse.py")
    cmd_param_build = ["python", param_parse_path, '--vehicle', vehicle]

    _, sts = run_cmd_blocking("Building fresh params", cmd_param_build)
    if sts != 0:
        progress("Parameter build failed")
        sys.exit(1)


def do_build(vehicledir, opts, frame_options):
    """Build build target (e.g. sitl) in directory vehicledir"""

    if opts.build_system == 'waf':
        return do_build_waf(opts, frame_options)

    old_dir = os.getcwd()

    os.chdir(vehicledir)

    if opts.clean:
        run_cmd_blocking("Building clean", ["make", "clean"])

    build_target = frame_options["make_target"]
    if opts.debug:
        build_target += "-debug"

    build_cmd = ["make", build_target]
    if opts.jobs is not None:
        build_cmd += ['-j', str(opts.jobs)]

    _, sts = run_cmd_blocking("Building %s" % build_target, build_cmd)
    if sts != 0:
        progress("Build failed; cleaning and rebuilding")
        run_cmd_blocking("Cleaning", ["make", "clean"])
        _, sts = run_cmd_blocking("Building %s" % build_target, build_cmd)
        if sts != 0:
            progress("Build failed")
            sys.exit(1)

    os.chdir(old_dir)


def get_user_locations_path():
    '''The user locations.txt file is located by default in
    $XDG_CONFIG_DIR/ardupilot/locations.txt. If $XDG_CONFIG_DIR is
    not defined, we look in $HOME/.config/ardupilot/locations.txt.  If
    $HOME is not defined, we look in ./.config/ardpupilot/locations.txt.'''

    config_dir = os.environ.get(
        'XDG_CONFIG_DIR',
        os.path.join(os.environ.get('HOME', '.'), '.config'))

    user_locations_path = os.path.join(
        config_dir, 'ardupilot', 'locations.txt')

    return user_locations_path


def find_location_by_name(autotest, locname):
    """Search locations.txt for locname, return GPS coords"""
    locations_userpath = os.environ.get('ARDUPILOT_LOCATIONS',
                                        get_user_locations_path())
    locations_filepath = os.path.join(autotest, "locations.txt")
    comment_regex = re.compile("\s*#.*")
    for path in [locations_userpath, locations_filepath]:
        if not os.path.isfile(path):
            continue

        with open(path, 'r') as fd:
            for line in fd:
                line = re.sub(comment_regex, "", line)
                line = line.rstrip("\n")
                if len(line) == 0:
                    continue
                (name, loc) = line.split("=")
                if name == locname:
                    return loc

    print("Failed to find location (%s)" % cmd_opts.location)
    sys.exit(1)


def progress_cmd(what, cmd):
    """Print cmd in a way a user could cut-and-paste to get the same effect"""
    progress(what)
    shell_text = "%s" % (" ".join(['"%s"' % x for x in cmd]))
    progress(shell_text)


def run_cmd_blocking(what, cmd, quiet=False, check=False, **kw):
    if not quiet:
        progress_cmd(what, cmd)

    try:
        p = subprocess.Popen(cmd, **kw)
        ret = os.waitpid(p.pid, 0)
    except Exception as e:
        print("[%s] An exception has occurred with command: '%s'" % (what, (' ').join(cmd)))
        print(e)
        sys.exit(1)

    _, sts = ret
    if check and sts != 0:
        progress("(%s) exited with code %d" % (what, sts,))
        sys.exit(1)
    return ret


def run_in_terminal_window(autotest, name, cmd):

    """Execute the run_in_terminal_window.sh command for cmd"""
    global windowID
    runme = [os.path.join(autotest, "run_in_terminal_window.sh"), name]
    runme.extend(cmd)
    progress_cmd("Run " + name, runme)

    if under_macos() and os.environ.get('DISPLAY'):
        # on MacOS record the window IDs so we can close them later
        out = subprocess.Popen(runme, stdout=subprocess.PIPE).communicate()[0]
        out = out.decode('utf-8')
        import re
        p = re.compile('tab 1 of window id (.*)')

        tstart = time.time()
        while time.time() - tstart < 5:
            tabs = p.findall(out)

            if len(tabs) > 0:
                break

            time.sleep(0.1)
        # sleep for extra 2 seconds for application to start
        time.sleep(2)
        if len(tabs) > 0:
            windowID.append(tabs[0])
        else:
            progress("Cannot find %s process terminal" % name)
    else:
        p = subprocess.Popen(runme)


tracker_uarta = None  # blemish


def start_antenna_tracker(autotest, opts):
    """Compile and run the AntennaTracker, add tracker to mavproxy"""

    global tracker_uarta
    progress("Preparing antenna tracker")
    tracker_home = find_location_by_name(find_autotest_dir(),
                                         opts.tracker_location)
    vehicledir = os.path.join(autotest, "../../" + "AntennaTracker")
    options = vinfo.options["AntennaTracker"]
    tracker_default_frame = options["default_frame"]
    tracker_frame_options = options["frames"][tracker_default_frame]
    do_build(vehicledir, opts, tracker_frame_options)
    tracker_instance = 1
    oldpwd = os.getcwd()
    os.chdir(vehicledir)
    tracker_uarta = "tcp:127.0.0.1:" + str(5760 + 10 * tracker_instance)
    exe = os.path.join(vehicledir, "AntennaTracker.elf")
    run_in_terminal_window(autotest,
                           "AntennaTracker",
                           ["nice",
                            exe,
                            "-I" + str(tracker_instance),
                            "--model=tracker",
                            "--home=" + tracker_home])
    os.chdir(oldpwd)


def start_vehicle(binary, autotest, opts, stuff, loc):
    """Run the ArduPilot binary"""

    cmd_name = opts.vehicle
    cmd = []
    if opts.valgrind:
        cmd_name += " (valgrind)"
        cmd.append("valgrind")
        # adding this option allows valgrind to cope with the overload
        # of operator new
        cmd.append("--soname-synonyms=somalloc=nouserintercepts")
    if opts.callgrind:
        cmd_name += " (callgrind)"
        cmd.append("valgrind")
        cmd.append("--tool=callgrind")
    if opts.gdb or opts.gdb_stopped:
        cmd_name += " (gdb)"
        cmd.append("gdb")
        gdb_commands_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        atexit.register(os.unlink, gdb_commands_file.name)

        for breakpoint in opts.breakpoint:
            gdb_commands_file.write("b %s\n" % (breakpoint,))
        if not opts.gdb_stopped:
            gdb_commands_file.write("r\n")
        gdb_commands_file.close()
        cmd.extend(["-x", gdb_commands_file.name])
        cmd.append("--args")
    if opts.strace:
        cmd_name += " (strace)"
        cmd.append("strace")
        strace_options = ['-o', binary + '.strace', '-s', '8000', '-ttt']
        cmd.extend(strace_options)

    cmd.append(binary)
    cmd.append("-S")
    cmd.append("-I" + str(opts.instance))
    cmd.extend(["--home", loc])
    if opts.wipe_eeprom:
        cmd.append("-w")
    cmd.extend(["--model", stuff["model"]])
    cmd.extend(["--speedup", str(opts.speedup)])
    if opts.sitl_instance_args:
        # this could be a lot better:
        cmd.extend(opts.sitl_instance_args.split(" "))
    if opts.mavlink_gimbal:
        cmd.append("--gimbal")
    path = None
    if "default_params_filename" in stuff:
        paths = stuff["default_params_filename"]
        if not isinstance(paths, list):
            paths = [paths]
        paths = [os.path.join(autotest, x) for x in paths]
        for x in paths:
            if not os.path.isfile(x):
                print("The parameter file (%s) does not exist" % (x,))
                sys.exit(1)
        path = ",".join(paths)
        progress("Using defaults from (%s)" % (path,))
    if opts.add_param_file:
        if not os.path.isfile(opts.add_param_file):
            print("The parameter file (%s) does not exist" %
                  (opts.add_param_file,))
            sys.exit(1)
        path += "," + str(opts.add_param_file)
        progress("Adding parameters from (%s)" % (str(opts.add_param_file),))
    if path is not None:
        cmd.extend(["--defaults", path])

    run_in_terminal_window(autotest, cmd_name, cmd)


def start_mavproxy(opts, stuff):
    """Run mavproxy"""
    # FIXME: would be nice to e.g. "mavproxy.mavproxy(....).run"
    # rather than shelling out

    extra_cmd = ""
    cmd = []
    if under_cygwin():
        cmd.append("/usr/bin/cygstart")
        cmd.append("-w")
        cmd.append("mavproxy.exe")
    else:
        cmd.append("mavproxy.py")

    if opts.hil:
        cmd.extend(["--load-module", "HIL"])
    else:
        cmd.extend(["--master", mavlink_port])
        if stuff["sitl-port"]:
            cmd.extend(["--sitl", simout_port])

    if not opts.no_extra_ports:
        ports = [p + 10 * cmd_opts.instance for p in [14550, 14551]]
        for port in ports:
            if os.path.isfile("/ardupilot.vagrant"):
                # We're running inside of a vagrant guest; forward our
                # mavlink out to the containing host OS
                cmd.extend(["--out", "10.0.2.2:" + str(port)])
            else:
                cmd.extend(["--out", "127.0.0.1:" + str(port)])

    if opts.tracker:
        cmd.extend(["--load-module", "tracker"])
        global tracker_uarta
        # tracker_uarta is set when we start the tracker...
        extra_cmd += ("module load map;"
                      "tracker set port %s; "
                      "tracker start; "
                      "tracker arm;" % (tracker_uarta,))

    if opts.mavlink_gimbal:
        cmd.extend(["--load-module", "gimbal"])

    if "extra_mavlink_cmds" in stuff:
        extra_cmd += " " + stuff["extra_mavlink_cmds"]

    # Parsing the arguments to pass to mavproxy, split args on space
    # and "=" signs and ignore those signs within quotation marks
    if opts.mavproxy_args:
        # It would be great if this could be done with regex
        mavargs = opts.mavproxy_args.split(" ")
        # Find the arguments with '=' in them and split them up
        for i, x in enumerate(mavargs):
            if '=' in x:
                mavargs[i] = x.split('=')[0]
                mavargs.insert(i+1, x.split('=')[1])
        # Use this flag to tell if parsing character inbetween a pair
        # of quotation marks
        inString = False
        beginStringIndex = []
        endStringIndex = []
        # Iterate through the arguments, looking for the arguments
        # that begin with a quotation mark and the ones that end with
        # a quotation mark
        for i, x in enumerate(mavargs):
            if not inString and x[0] == "\"":
                beginStringIndex.append(i)
                mavargs[i] = x[1:]
                inString = True
            elif inString and x[-1] == "\"":
                endStringIndex.append(i)
                inString = False
                mavargs[i] = x[:-1]
        # Replace the list items with one string to be passed into mavproxy
        for begin, end in zip(beginStringIndex, endStringIndex):
            replacement = " ".join(mavargs[begin:end+1])
            mavargs[begin] = replacement
            mavargs = mavargs[0:begin+1] + mavargs[end+1:]
        cmd.extend(mavargs)

    # compatibility pass-through parameters (for those that don't want
    # to use -C :-)
    for out in opts.out:
        cmd.extend(['--out', out])
    if opts.map:
        cmd.append('--map')
    if opts.console:
        cmd.append('--console')
    if opts.aircraft is not None:
        cmd.extend(['--aircraft', opts.aircraft])

    if opts.fresh_params:
        # these were built earlier:
        path = os.path.join(os.getcwd(), "apm.pdef.xml")
        cmd.extend(['--load-module', 'param:{"xml-filepath":"%s"}' % path])

    if len(extra_cmd):
        cmd.extend(['--cmd', extra_cmd])

    local_mp_modules_dir = os.path.abspath(
        os.path.join(__file__, '..', '..', 'mavproxy_modules'))
    env = dict(os.environ)
    env['PYTHONPATH'] = (local_mp_modules_dir +
                         os.pathsep +
                         env.get('PYTHONPATH', ''))

    run_cmd_blocking("Run MavProxy", cmd, env=env)
    progress("MAVProxy exited")


vehicle_options_string = '|'.join(vinfo.options.keys())


def generate_frame_help():
    ret = ""
    for vehicle in vinfo.options:
        frame_options = vinfo.options[vehicle]["frames"].keys()
        frame_options_string = '|'.join(frame_options)
        ret += "%s: %s\n" % (vehicle, frame_options_string)
    return ret


# define and run parser
parser = CompatOptionParser(
    "sim_vehicle.py",
    epilog=""
    "eeprom.bin in the starting directory contains the parameters for your"
    "simulated vehicle. Always start from the same directory. It is "
    "recommended that you start in the main vehicle directory for the vehicle"
    "you are simulating, for example, start in the ArduPlane directory to "
    "simulate ArduPlane")

parser.add_option("-v", "--vehicle",
                  type='choice',
                  default=None,
                  help="vehicle type (%s)" % vehicle_options_string,
                  choices=list(vinfo.options.keys()))
parser.add_option("-f", "--frame", type='string', default=None, help="""set vehicle frame type

%s""" % (generate_frame_help()))
parser.add_option("-C", "--sim_vehicle_sh_compatible",
                  action='store_true',
                  default=False,
                  help="be compatible with the way sim_vehicle.sh works; "
                  "make this the first option")
parser.add_option("-H", "--hil",
                  action='store_true',
                  default=False,
                  help="start HIL")

group_build = optparse.OptionGroup(parser, "Build options")
group_build.add_option("-N", "--no-rebuild",
                       action='store_true',
                       default=False,
                       help="don't rebuild before starting ardupilot")
group_build.add_option("-D", "--debug",
                       action='store_true',
                       default=False,
                       help="build with debugging")
group_build.add_option("-c", "--clean",
                       action='store_true',
                       default=False,
                       help="do a make clean before building")
group_build.add_option("-j", "--jobs",
                       default=None,
                       type='int',
                       help="number of processors to use during build "
                       "(default for waf : number of processor, for make : 1)")
group_build.add_option("-b", "--build-target",
                       default=None,
                       type='string',
                       help="override SITL build target")
group_build.add_option("-s", "--build-system",
                       default="waf",
                       type='choice',
                       choices=["make", "waf"],
                       help="build system to use")
group_build.add_option("", "--rebuild-on-failure",
                       dest="rebuild_on_failure",
                       action='store_true',
                       default=False,
                       help="if build fails, do not clean and rebuild")
group_build.add_option("", "--waf-configure-arg",
                       action="append",
                       dest="waf_configure_args",
                       type="string",
                       default=[],
                       help="extra arguments to pass to waf in configure step")
group_build.add_option("", "--waf-build-arg",
                       action="append",
                       dest="waf_build_args",
                       type="string",
                       default=[],
                       help="extra arguments to pass to waf in its build step")
parser.add_option_group(group_build)

group_sim = optparse.OptionGroup(parser, "Simulation options")
group_sim.add_option("-I", "--instance",
                     default=0,
                     type='int',
                     help="instance of simulator")
group_sim.add_option("-V", "--valgrind",
                     action='store_true',
                     default=False,
                     help="enable valgrind for memory access checking (slow!)")
group_sim.add_option("", "--callgrind",
                     action='store_true',
                     default=False,
                     help="enable valgrind for performance analysis (slow!!)")
group_sim.add_option("-T", "--tracker",
                     action='store_true',
                     default=False,
                     help="start an antenna tracker instance")
group_sim.add_option("-A", "--sitl-instance-args",
                     type='string',
                     default=None,
                     help="pass arguments to SITL instance")
group_sim.add_option("-G", "--gdb",
                     action='store_true',
                     default=False,
                     help="use gdb for debugging ardupilot")
group_sim.add_option("-g", "--gdb-stopped",
                     action='store_true',
                     default=False,
                     help="use gdb for debugging ardupilot (no auto-start)")
group_sim.add_option("-d", "--delay-start",
                     default=0,
                     type='float',
                     help="delay start of mavproxy by this number of seconds")
group_sim.add_option("-B", "--breakpoint",
                     type='string',
                     action="append",
                     default=[],
                     help="add a breakpoint at given location in debugger")
group_sim.add_option("-M", "--mavlink-gimbal",
                     action='store_true',
                     default=False,
                     help="enable MAVLink gimbal")
group_sim.add_option("-L", "--location", type='string',
                     default='CMAC',
                     help="use start location from "
                     "Tools/autotest/locations.txt")
group_sim.add_option("-l", "--custom-location",
                     type='string',
                     default=None,
                     help="set custom start location")
group_sim.add_option("-S", "--speedup",
                     default=1,
                     type='int',
                     help="set simulation speedup (1 for wall clock time)")
group_sim.add_option("-t", "--tracker-location",
                     default='CMAC_PILOTSBOX',
                     type='string',
                     help="set antenna tracker start location")
group_sim.add_option("-w", "--wipe-eeprom",
                     action='store_true',
                     default=False, help="wipe EEPROM and reload parameters")
group_sim.add_option("-m", "--mavproxy-args",
                     default=None,
                     type='string',
                     help="additional arguments to pass to mavproxy.py")
group_sim.add_option("", "--strace",
                     action='store_true',
                     default=False,
                     help="strace the ArduPilot binary")
group_sim.add_option("", "--model",
                     type='string',
                     default=None,
                     help="Override simulation model to use")
group_sim.add_option("", "--use-dir",
                     type='string',
                     default=None,
                     help="Store SITL state and output in named directory")
group_sim.add_option("", "--no-mavproxy",
                     action='store_true',
                     default=False,
                     help="Don't launch MAVProxy")
group_sim.add_option("", "--fresh-params",
                     action='store_true',
                     dest='fresh_params',
                     default=False,
                     help="Generate and use local parameter help XML")
group_sim.add_option("", "--osd",
                     action='store_true',
                     dest='OSD',
                     default=False,
                     help="Enable SITL OSD")
group_sim.add_option("", "--add-param-file",
                     type='string',
                     default=None,
                     help="Add a parameters file to use")
group_sim.add_option("", "--no-extra-ports",
                     action='store_true',
                     dest='no_extra_ports',
                     default=False,
                     help="Disable setup of UDP 14550 and 14551 output")
parser.add_option_group(group_sim)


# special-cased parameters for mavproxy, because some people's fingers
# have long memories, and they don't want to use -C :-)
group = optparse.OptionGroup(parser,
                             "Compatibility MAVProxy options "
                             "(consider using --mavproxy-args instead)")
group.add_option("", "--out",
                 default=[],
                 type='string',
                 action="append",
                 help="create an additional mavlink output")
group.add_option("", "--map",
                 default=False,
                 action='store_true',
                 help="load map module on startup")
group.add_option("", "--console",
                 default=False,
                 action='store_true',
                 help="load console module on startup")
group.add_option("", "--aircraft",
                 default=None,
                 help="store state and logs in named directory")
parser.add_option_group(group)

cmd_opts, cmd_args = parser.parse_args()

# clean up processes at exit:
atexit.register(kill_tasks)

progress("Start")

if cmd_opts.sim_vehicle_sh_compatible and cmd_opts.jobs is None:
    cmd_opts.jobs = 1

# validate parameters
if cmd_opts.hil:
    if cmd_opts.valgrind:
        print("May not use valgrind with hil")
        sys.exit(1)
    if cmd_opts.callgrind:
        print("May not use callgrind with hil")
        sys.exit(1)
    if cmd_opts.gdb or cmd_opts.gdb_stopped:
        print("May not use gdb with hil")
        sys.exit(1)
    if cmd_opts.strace:
        print("May not use strace with hil")
        sys.exit(1)

if cmd_opts.valgrind and (cmd_opts.gdb or cmd_opts.gdb_stopped):
    print("May not use valgrind with gdb")
    sys.exit(1)

if cmd_opts.valgrind and cmd_opts.callgrind:
    print("May not use valgrind with callgrind")
    sys.exit(1)

if cmd_opts.strace and (cmd_opts.gdb or cmd_opts.gdb_stopped):
    print("May not use strace with gdb")
    sys.exit(1)

if cmd_opts.strace and cmd_opts.valgrind:
    print("valgrind and strace almost certainly not a good idea")

if cmd_opts.strace and cmd_opts.callgrind:
    print("callgrind and strace almost certainly not a good idea")

# magically determine vehicle type (if required):
if cmd_opts.vehicle is None:
    cwd = os.getcwd()
    cmd_opts.vehicle = os.path.basename(cwd)

if cmd_opts.vehicle not in vinfo.options:
    # try in parent directories, useful for having config in subdirectories
    cwd = os.getcwd()
    while cwd:
        bname = os.path.basename(cwd)
        if not bname:
            break
        if bname in vinfo.options:
            cmd_opts.vehicle = bname
            break
        cwd = os.path.dirname(cwd)

# try to validate vehicle
if cmd_opts.vehicle not in vinfo.options:
    progress('''
** Is (%s) really your vehicle type?
Perhaps you could try -v %s
You could also try changing directory to e.g. the ArduCopter subdirectory
''' % (cmd_opts.vehicle, vehicle_options_string))
    sys.exit(1)

# determine frame options (e.g. build type might be "sitl")
if cmd_opts.frame is None:
    cmd_opts.frame = vinfo.options[cmd_opts.vehicle]["default_frame"]

# setup ports for this instance
mavlink_port = "tcp:127.0.0.1:" + str(5760 + 10 * cmd_opts.instance)
simout_port = "127.0.0.1:" + str(5501 + 10 * cmd_opts.instance)

frame_infos = vinfo.options_for_frame(cmd_opts.frame,
                                      cmd_opts.vehicle,
                                      cmd_opts)

vehicle_dir = os.path.realpath(os.path.join(find_root_dir(), cmd_opts.vehicle))
if not os.path.exists(vehicle_dir):
    print("vehicle directory (%s) does not exist" % (vehicle_dir,))
    sys.exit(1)

if not cmd_opts.hil:
    if cmd_opts.instance == 0:
        kill_tasks()

if cmd_opts.tracker:
    start_antenna_tracker(find_autotest_dir(), cmd_opts)

if cmd_opts.custom_location:
    location = cmd_opts.custom_location
    progress("Starting up at %s" % (location,))
else:
    location = find_location_by_name(find_autotest_dir(), cmd_opts.location)
    progress("Starting up at %s (%s)" % (location, cmd_opts.location))

if cmd_opts.use_dir is not None:
    new_dir = cmd_opts.use_dir
    try:
        os.makedirs(os.path.realpath(new_dir))
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
    os.chdir(new_dir)

if cmd_opts.hil:
    # (unlikely)
    run_in_terminal_window(find_autotest_dir(),
                           "JSBSim",
                           [os.path.join(find_autotest_dir(),
                                         "jsb_sim/runsim.py"),
                            "--home", location,
                            "--speedup=" + str(cmd_opts.speedup)])
else:
    if not cmd_opts.no_rebuild:  # i.e. we should rebuild
        do_build(vehicle_dir, cmd_opts, frame_infos)

    if cmd_opts.fresh_params:
        do_build_parameters(cmd_opts.vehicle)

    if cmd_opts.build_system == "waf":
        binary_basedir = "build/sitl"
        vehicle_binary = os.path.join(find_root_dir(),
                                      binary_basedir,
                                      frame_infos["waf_target"])
    else:
        vehicle_binary = os.path.join(vehicle_dir, cmd_opts.vehicle + ".elf")

    if not os.path.exists(vehicle_binary):
        print("Vehicle binary (%s) does not exist" % (vehicle_binary,))
        sys.exit(1)

    start_vehicle(vehicle_binary,
                  find_autotest_dir(),
                  cmd_opts,
                  frame_infos,
                  location)

if cmd_opts.delay_start:
    progress("Sleeping for %f seconds" % (cmd_opts.delay_start,))
    time.sleep(float(cmd_opts.delay_start))

try:
    if cmd_opts.no_mavproxy:
        time.sleep(3)  # output our message after run_in_terminal_window.sh's
        progress("Waiting for SITL to exit")
        wait_unlimited()
    else:
        start_mavproxy(cmd_opts, frame_infos)
except KeyboardInterrupt:
    progress("Keyboard Interrupt received ...")

sys.exit(0)
