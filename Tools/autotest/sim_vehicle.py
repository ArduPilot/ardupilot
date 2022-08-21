#!/usr/bin/env python

"""
Framework to start a simulated vehicle and connect it to MAVProxy.

Peter Barker, April 2016
based on sim_vehicle.sh by Andrew Tridgell, October 2011

AP_FLAKE8_CLEAN

"""
from __future__ import print_function

import atexit
import datetime
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
import binascii
import math

from pysim import util
from pysim import vehicleinfo


# List of open terminal windows for macosx
windowID = []

autotest_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(autotest_dir, '../..'))

try:
    from pymavlink import mavextra
except ImportError:
    sys.path.append(os.path.join(root_dir, "modules/mavlink"))
    from pymavlink import mavextra

os.environ["SIM_VEHICLE_SESSION"] = binascii.hexlify(os.urandom(8)).decode()


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
    output_lines = pipe.stdout.read().decode('utf-8').replace("\r", "").split("\n")
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
            except Exception:
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
        pdict = proc.as_dict(attrs=['environ', 'status'])
        if pdict['status'] == psutil.STATUS_ZOMBIE:
            continue
        if pdict['environ'] is not None:
            if pdict['environ'].get('SIM_VEHICLE_SESSION') == os.environ['SIM_VEHICLE_SESSION']:
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

    if cmd_opts.coverage:
        import psutil
        for proc in psutil.process_iter(['pid', 'name', 'environ']):
            if proc.name() not in ["arducopter", "ardurover", "arduplane", "ardusub", "antennatracker"]:
                # only kill vehicle that way
                continue
            if os.environ['SIM_VEHICLE_SESSION'] not in proc.environ().get('SIM_VEHICLE_SESSION'):
                # only kill vehicle launched with sim_vehicle.py that way
                continue
            proc.terminate()
            progress("Waiting SITL to exit cleanly and write coverage .gcda")
            try:
                proc.wait(timeout=30)
                progress("Done")
            except psutil.TimeoutExpired:
                progress("SITL doesn't want to exit cleaning, killing ...")
                proc.kill()

    try:
        victim_names = {
            'JSBSim',
            'lt-JSBSim',
            'ArduPlane.elf',
            'ArduCopter.elf',
            'ArduSub.elf',
            'Rover.elf',
            'AntennaTracker.elf',
            'JSBSIm.exe',
            'MAVProxy.exe',
            'runsim.py',
            'AntennaTracker.elf',
            'scrimmage',
            'ardurover',
            'arduplane',
            'arducopter'
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


def wait_unlimited():
    """Wait until signal received"""
    while True:
        time.sleep(600)


vinfo = vehicleinfo.VehicleInfo()


def do_build(opts, frame_options):
    """Build sitl using waf"""
    progress("WAF build")

    old_dir = os.getcwd()
    os.chdir(root_dir)

    waf_light = os.path.join(root_dir, "modules/waf/waf-light")

    configure_target = frame_options.get('configure_target', 'sitl')

    cmd_configure = [waf_light, "configure", "--board", configure_target]
    if opts.debug:
        cmd_configure.append("--debug")

    if opts.coverage:
        cmd_configure.append("--coverage")

    if opts.enable_onvif and 'antennatracker' in frame_options["waf_target"]:
        cmd_configure.append("--enable-onvif")

    if opts.OSD:
        cmd_configure.append("--enable-sfml")
        cmd_configure.append("--sitl-osd")

    if opts.OSDMSP:
        cmd_configure.append("--osd")

    if opts.rgbled:
        cmd_configure.append("--enable-sfml")
        cmd_configure.append("--sitl-rgbled")

    if opts.tonealarm:
        cmd_configure.append("--enable-sfml-audio")

    if opts.math_check_indexes:
        cmd_configure.append("--enable-math-check-indexes")

    if opts.disable_ekf2:
        cmd_configure.append("--disable-ekf2")

    if opts.disable_ekf3:
        cmd_configure.append("--disable-ekf3")

    if opts.postype_single:
        cmd_configure.append("--postype-single")

    if opts.ekf_double:
        cmd_configure.append("--ekf-double")

    if opts.ekf_single:
        cmd_configure.append("--ekf-single")

    if opts.sitl_32bit:
        cmd_configure.append("--sitl-32bit")

    if opts.ubsan:
        cmd_configure.append("--ubsan")

    if opts.ubsan_abort:
        cmd_configure.append("--ubsan-abort")

    for nv in opts.define:
        cmd_configure.append("--define=%s" % nv)

    pieces = [shlex.split(x) for x in opts.waf_configure_args]
    for piece in pieces:
        cmd_configure.extend(piece)

    run_cmd_blocking("Configure waf", cmd_configure, check=True)

    if opts.clean:
        run_cmd_blocking("Building clean", [waf_light, "clean"])

    print(frame_options)
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
        autotest_dir, "param_metadata/param_parse.py")
    cmd_param_build = ["python", param_parse_path, '--vehicle', vehicle]

    _, sts = run_cmd_blocking("Building fresh params", cmd_param_build)
    if sts != 0:
        progress("Parameter build failed")
        sys.exit(1)


def get_user_locations_path():
    '''The user locations.txt file is located by default in
    $XDG_CONFIG_DIR/ardupilot/locations.txt. If $XDG_CONFIG_DIR is
    not defined, we look in $HOME/.config/ardupilot/locations.txt.  If
    $HOME is not defined, we look in ./.config/ardupilot/locations.txt.'''

    config_dir = os.environ.get(
        'XDG_CONFIG_DIR',
        os.path.join(os.environ.get('HOME', '.'), '.config'))

    user_locations_path = os.path.join(
        config_dir, 'ardupilot', 'locations.txt')

    return user_locations_path


def find_offsets(instances, file_path):
    offsets = {}
    swarminit_filepath = os.path.join(autotest_dir, "swarminit.txt")
    comment_regex = re.compile(r"\s*#.*")
    for path in [file_path, swarminit_filepath]:
        if os.path.isfile(path):
            with open(path, 'r') as fd:
                for line in fd:
                    line = re.sub(comment_regex, "", line)
                    line = line.rstrip("\n")
                    if len(line) == 0:
                        continue
                    (instance, offset) = line.split("=")
                    instance = (int)(instance)
                    if (instance not in offsets) and (instance in instances):
                        offsets[instance] = [(float)(x) for x in offset.split(",")]
                        continue
                    if len(offsets) == len(instances):
                        return offsets
        if len(offsets) == len(instances):
            return offsets
    for instance in instances:
        if instance not in offsets:
            offsets[instance] = [90.0, 20.0 * instance, 0.0, None]
    return offsets


def find_geocoder_location(locname):
    '''find a location using geocoder and SRTM'''
    try:
        import geocoder
    except ImportError:
        print("geocoder not installed")
        return None
    j = geocoder.osm(locname)
    if j is None or not hasattr(j, 'lat') or j.lat is None:
        print("geocoder failed to find '%s'" % locname)
        return None
    lat = j.lat
    lon = j.lng
    from MAVProxy.modules.mavproxy_map import srtm
    downloader = srtm.SRTMDownloader()
    downloader.loadFileList()
    start = time.time()
    alt = None
    while time.time() - start < 5:
        tile = downloader.getTile(int(math.floor(lat)), int(math.floor(lon)))
        if tile:
            alt = tile.getAltitudeFromLatLon(lat, lon)
            break
    if alt is None:
        print("timed out getting altitude for '%s'" % locname)
        return None
    return [lat, lon, alt, 0.0]


def find_location_by_name(locname):
    """Search locations.txt for locname, return GPS coords"""
    locations_userpath = os.environ.get('ARDUPILOT_LOCATIONS',
                                        get_user_locations_path())
    locations_filepath = os.path.join(autotest_dir, "locations.txt")
    comment_regex = re.compile(r"\s*#.*")
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
                    return [(float)(x) for x in loc.split(",")]

    # fallback to geocoder if available
    loc = find_geocoder_location(locname)
    if loc is None:
        sys.exit(1)
    return loc


def find_spawns(loc, offsets):
    lat, lon, alt, heading = loc
    spawns = {}
    for k in offsets:
        (x, y, z, head) = offsets[k]
        if head is None:
            head = heading
        g = mavextra.gps_offset(lat, lon, x, y)
        spawns[k] = ",".join([str(g[0]), str(g[1]), str(alt+z), str(head)])
    return spawns


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


def run_in_terminal_window(name, cmd, **kw):

    """Execute the run_in_terminal_window.sh command for cmd"""
    global windowID
    runme = [os.path.join(autotest_dir, "run_in_terminal_window.sh"), name]
    runme.extend(cmd)
    progress_cmd("Run " + name, runme)

    if under_macos() and os.environ.get('DISPLAY'):
        # on MacOS record the window IDs so we can close them later
        out = subprocess.Popen(runme, stdout=subprocess.PIPE, **kw).communicate()[0]
        out = out.decode('utf-8')
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
        subprocess.Popen(runme, **kw)


tracker_uarta = None  # blemish


def start_antenna_tracker(opts):
    """Compile and run the AntennaTracker, add tracker to mavproxy"""

    global tracker_uarta
    progress("Preparing antenna tracker")
    tracker_home = find_location_by_name(opts.tracker_location)
    vehicledir = os.path.join(autotest_dir, "../../" + "AntennaTracker")
    options = vinfo.options["AntennaTracker"]
    tracker_default_frame = options["default_frame"]
    tracker_frame_options = options["frames"][tracker_default_frame]
    do_build(opts, tracker_frame_options)
    tracker_instance = 1
    oldpwd = os.getcwd()
    os.chdir(vehicledir)
    tracker_uarta = "tcp:127.0.0.1:" + str(5760 + 10 * tracker_instance)
    if cmd_opts.build_system == "waf":
        binary_basedir = "build/sitl"
        exe = os.path.join(root_dir,
                           binary_basedir,
                           "bin/antennatracker")
    else:
        exe = os.path.join(vehicledir, "AntennaTracker.elf")
    run_in_terminal_window("AntennaTracker",
                           ["nice",
                            exe,
                            "-I" + str(tracker_instance),
                            "--model=tracker",
                            "--home=" + ",".join([str(x) for x in tracker_home])])
    os.chdir(oldpwd)


def start_CAN_GPS(opts):
    """Compile and run the sitl_periph_gps"""

    global can_uarta
    progress("Preparing sitl_periph_gps")
    options = vinfo.options["sitl_periph_gps"]['frames']['gps']
    do_build(opts, options)
    exe = os.path.join(root_dir, 'build/sitl_periph_gps', 'bin/AP_Periph')
    run_in_terminal_window("sitl_periph_gps",
                           ["nice", exe])


def start_vehicle(binary, opts, stuff, spawns=None):
    """Run the ArduPilot binary"""

    cmd_name = opts.vehicle
    cmd = []
    if opts.valgrind:
        cmd_name += " (valgrind)"
        cmd.append("valgrind")
        # adding this option allows valgrind to cope with the overload
        # of operator new
        cmd.append("--soname-synonyms=somalloc=nouserintercepts")
        cmd.append("--track-origins=yes")
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
        if opts.disable_breakpoints:
            gdb_commands_file.write("disable\n")
        gdb_commands_file.write("set pagination off\n")
        if not opts.gdb_stopped:
            gdb_commands_file.write("r\n")
        gdb_commands_file.close()
        cmd.extend(["-x", gdb_commands_file.name])
        cmd.append("--args")
    elif opts.lldb or opts.lldb_stopped:
        cmd_name += " (lldb)"
        cmd.append("lldb")
        lldb_commands_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        atexit.register(os.unlink, lldb_commands_file.name)

        for breakpoint in opts.breakpoint:
            lldb_commands_file.write("b %s\n" % (breakpoint,))
        if not opts.lldb_stopped:
            lldb_commands_file.write("process launch\n")
        lldb_commands_file.close()
        cmd.extend(["-s", lldb_commands_file.name])
        cmd.append("--")
    if opts.strace:
        cmd_name += " (strace)"
        cmd.append("strace")
        strace_options = ['-o', binary + '.strace', '-s', '8000', '-ttt']
        cmd.extend(strace_options)

    cmd.append(binary)
    cmd.append("-S")
    if opts.wipe_eeprom:
        cmd.append("-w")
    cmd.extend(["--model", stuff["model"]])
    cmd.extend(["--speedup", str(opts.speedup)])
    if opts.sysid is not None:
        cmd.extend(["--sysid", str(opts.sysid)])
    if opts.slave is not None:
        cmd.extend(["--slave", str(opts.slave)])
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
        paths = [util.relcurdir(os.path.join(autotest_dir, x)) for x in paths]
        for x in paths:
            if not os.path.isfile(x):
                print("The parameter file (%s) does not exist" % (x,))
                sys.exit(1)
        path = ",".join(paths)
        progress("Using defaults from (%s)" % (path,))
    if opts.flash_storage:
        cmd.append("--set-storage-flash-enabled 1")
        cmd.append("--set-storage-posix-enabled 0")
    elif opts.fram_storage:
        cmd.append("--set-storage-fram-enabled 1")
        cmd.append("--set-storage-posix-enabled 0")
    if opts.add_param_file:
        for file in opts.add_param_file:
            if not os.path.isfile(file):
                print("The parameter file (%s) does not exist" %
                      (file,))
                sys.exit(1)

            if path is not None:
                path += "," + str(file)
            else:
                path = str(file)

            progress("Adding parameters from (%s)" % (str(file),))
    if opts.OSDMSP:
        path += "," + os.path.join(root_dir, "libraries/AP_MSP/Tools/osdtest.parm")
        path += "," + os.path.join(autotest_dir, "default_params/msposd.parm")
        subprocess.Popen([os.path.join(root_dir, "libraries/AP_MSP/Tools/msposd.py")])

    if path is not None:
        cmd.extend(["--defaults", path])

    if cmd_opts.start_time is not None:
        # Parse start_time into a double precision number specifying seconds since 1900.
        try:
            start_time_UTC = time.mktime(datetime.datetime.strptime(cmd_opts.start_time, '%Y-%m-%d-%H:%M').timetuple())
        except Exception:
            print("Incorrect start time format - require YYYY-MM-DD-HH:MM (given %s)" % cmd_opts.start_time)
            sys.exit(1)

        cmd.append("--start-time=%d" % start_time_UTC)

    old_dir = os.getcwd()
    for i, i_dir in zip(instances, instance_dir):
        c = ["-I" + str(i)]
        if spawns is not None:
            c.extend(["--home", spawns[i]])
        if opts.mcast:
            c.extend(["--uartA", "mcast:"])
        elif opts.udp:
            c.extend(["--uartA", "udpclient:127.0.0.1:" + str(5760+i*10)])
        if opts.auto_sysid:
            if opts.sysid is not None:
                raise ValueError("Can't use auto-sysid and sysid together")
            sysid = i + 1
            # Take 0-based logging into account
            if sysid < 1 or sysid > 255:
                raise ValueError("Invalid system id %d" % sysid)
            c.extend(["--sysid", str(sysid)])

        os.chdir(i_dir)
        run_in_terminal_window(cmd_name, cmd + c)
    os.chdir(old_dir)


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

    if opts.mcast:
        cmd.extend(["--master", "mcast:"])

    for i in instances:
        if not opts.no_extra_ports:
            ports = [p + 10 * i for p in [14550, 14551]]
            for port in ports:
                if os.path.isfile("/ardupilot.vagrant"):
                    # We're running inside of a vagrant guest; forward our
                    # mavlink out to the containing host OS
                    cmd.extend(["--out", "10.0.2.2:" + str(port)])
                else:
                    cmd.extend(["--out", "127.0.0.1:" + str(port)])

        if not opts.mcast:
            if opts.udp:
                cmd.extend(["--master", ":" + str(5760 + 10 * i)])
            else:
                cmd.extend(["--master", "tcp:127.0.0.1:" + str(5760 + 10 * i)])
        if stuff["sitl-port"] and not opts.no_rcin:
            cmd.extend(["--sitl", "127.0.0.1:" + str(5501 + 10 * i)])

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
    if opts.moddebug:
        cmd.append('--moddebug=%u' % opts.moddebug)

    if opts.fresh_params:
        # these were built earlier:
        path = os.path.join(os.getcwd(), "apm.pdef.xml")
        cmd.extend(['--load-module', 'param:{"xml-filepath":"%s"}' % path])

    if len(extra_cmd):
        cmd.extend(['--cmd', extra_cmd])

    # add Tools/mavproxy_modules to PYTHONPATH in autotest so we can
    # find random MAVProxy helper modules like sitl_calibration
    local_mp_modules_dir = os.path.abspath(
        os.path.join(__file__, '..', '..', 'mavproxy_modules'))
    env = dict(os.environ)
    old = env.get('PYTHONPATH', None)
    env['PYTHONPATH'] = local_mp_modules_dir
    if old is not None:
        env['PYTHONPATH'] += os.path.pathsep + old

    run_cmd_blocking("Run MavProxy", cmd, env=env)
    progress("MAVProxy exited")


vehicle_options_string = '|'.join(vinfo.options.keys())


def generate_frame_help():
    ret = ""
    for vehicle in vinfo.options:
        frame_options = sorted(vinfo.options[vehicle]["frames"].keys())
        frame_options_string = '|'.join(frame_options)
        ret += "%s: %s\n" % (vehicle, frame_options_string)
    return ret


# define and run parser
parser = CompatOptionParser(
    "sim_vehicle.py",
    epilog=""
    "eeprom.bin in the starting directory contains the parameters for your "
    "simulated vehicle. Always start from the same directory. It is "
    "recommended that you start in the main vehicle directory for the vehicle "
    "you are simulating, for example, start in the ArduPlane directory to "
    "simulate ArduPlane")

vehicle_choices = list(vinfo.options.keys())
# add an alias for people with too much m
vehicle_choices.append("APMrover2")
vehicle_choices.append("Copter")  # should change to ArduCopter at some stage
vehicle_choices.append("Plane")  # should change to ArduPlane at some stage
vehicle_choices.append("Sub")  # should change to Sub at some stage
vehicle_choices.append("copter")  # should change to ArduCopter at some stage
vehicle_choices.append("plane")  # should change to ArduPlane at some stage
vehicle_choices.append("sub")  # should change to Sub at some stage
vehicle_choices.append("blimp")  # should change to Blimp at some stage

parser.add_option("-v", "--vehicle",
                  type='choice',
                  default=None,
                  help="vehicle type (%s)" % vehicle_options_string,
                  choices=vehicle_choices)
parser.add_option("-f", "--frame", type='string', default=None, help="""set vehicle frame type

%s""" % (generate_frame_help()))

parser.add_option("--vehicle-binary",
                  default=None,
                  help="vehicle binary path")

parser.add_option("-C", "--sim_vehicle_sh_compatible",
                  action='store_true',
                  default=False,
                  help="be compatible with the way sim_vehicle.sh works; "
                  "make this the first option")

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
group_build.add_option("--enable-math-check-indexes",
                       default=False,
                       action="store_true",
                       dest="math_check_indexes",
                       help="enable checking of math indexes")
group_build.add_option("", "--sitl-32bit",
                       default=False,
                       action='store_true',
                       dest="sitl_32bit",
                       help="compile sitl using 32-bit")
group_build.add_option("", "--configure-define",
                       default=[],
                       action='append',
                       dest="define",
                       help="create a preprocessor define")
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
group_build.add_option("", "--coverage",
                       action='store_true',
                       default=False,
                       help="use coverage build")
group_build.add_option("", "--ubsan",
                       default=False,
                       action='store_true',
                       dest="ubsan",
                       help="compile sitl with undefined behaviour sanitiser")
group_build.add_option("", "--ubsan-abort",
                       default=False,
                       action='store_true',
                       dest="ubsan_abort",
                       help="compile sitl with undefined behaviour sanitiser and abort on error")
parser.add_option_group(group_build)

group_sim = optparse.OptionGroup(parser, "Simulation options")
group_sim.add_option("-I", "--instance",
                     default=0,
                     type='int',
                     help="instance of simulator")
group_sim.add_option("-n", "--count",
                     type='int',
                     default=1,
                     help="vehicle count; if this is specified, -I is used as a base-value")
group_sim.add_option("-i", "--instances",
                     default=None,
                     type='string',
                     help="a space delimited list of instances to spawn; if specified, overrides -I and -n.")
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
group_sim.add_option("", "--enable-onvif",
                     action="store_true",
                     help="enable onvif camera control sim using AntennaTracker")
group_sim.add_option("", "--can-gps",
                     action='store_true',
                     default=False,
                     help="start a DroneCAN GPS instance (use Tools/scripts/CAN/can_sitl_nodev.sh first)")
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
group_sim.add_option("--lldb",
                     action='store_true',
                     default=False,
                     help="use lldb for debugging ardupilot")
group_sim.add_option("--lldb-stopped",
                     action='store_true',
                     default=False,
                     help="use ldb for debugging ardupilot (no auto-start)")
group_sim.add_option("-d", "--delay-start",
                     default=0,
                     type='float',
                     help="delay start of mavproxy by this number of seconds")
group_sim.add_option("-B", "--breakpoint",
                     type='string',
                     action="append",
                     default=[],
                     help="add a breakpoint at given location in debugger")
group_sim.add_option("--disable-breakpoints",
                     default=False,
                     action='store_true',
                     help="disable all breakpoints before starting")
group_sim.add_option("-M", "--mavlink-gimbal",
                     action='store_true',
                     default=False,
                     help="enable MAVLink gimbal")
group_sim.add_option("-L", "--location", type='string',
                     default=None,
                     help="use start location from "
                     "Tools/autotest/locations.txt")
group_sim.add_option("-l", "--custom-location",
                     type='string',
                     default=None,
                     help="set custom start location (lat,lon,alt,heading)")
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
group_sim.add_option("", "--scrimmage-args",
                     default=None,
                     type='string',
                     help="arguments used to populate SCRIMMAGE mission (comma-separated). "
                     "Currently visual_model, motion_model, and terrain are supported. "
                     "Usage: [instance=]argument=value...")
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
group_sim.add_option("", "--mcast",
                     action="store_true",
                     default=False,
                     help="Use multicasting at default 239.255.145.50:14550")
group_sim.add_option("", "--udp",
                     action="store_true",
                     default=False,
                     help="Use UDP on 127.0.0.1:5760")
group_sim.add_option("", "--osd",
                     action='store_true',
                     dest='OSD',
                     default=False,
                     help="Enable SITL OSD")
group_sim.add_option("", "--osdmsp",
                     action='store_true',
                     dest='OSDMSP',
                     default=False,
                     help="Enable SITL OSD using MSP")
group_sim.add_option("", "--tonealarm",
                     action='store_true',
                     dest='tonealarm',
                     default=False,
                     help="Enable SITL ToneAlarm")
group_sim.add_option("", "--rgbled",
                     action='store_true',
                     dest='rgbled',
                     default=False,
                     help="Enable SITL RGBLed")
group_sim.add_option("", "--add-param-file",
                     type='string',
                     action="append",
                     default=None,
                     help="Add a parameters file to use")
group_sim.add_option("", "--no-extra-ports",
                     action='store_true',
                     dest='no_extra_ports',
                     default=False,
                     help="Disable setup of UDP 14550 and 14551 output")
group_sim.add_option("-Z", "--swarm",
                     type='string',
                     default=None,
                     help="Specify path of swarminit.txt for shifting spawn location")
group_sim.add_option("", "--auto-offset-line",
                     type="string",
                     default=None,
                     help="Argument of form  BEARING,DISTANCE.  When running multiple instances, form a line along bearing with an interval of DISTANCE",  # NOQA
                     )
group_sim.add_option("--flash-storage",
                     action='store_true',
                     help="use flash storage emulation")
group_sim.add_option("--fram-storage",
                     action='store_true',
                     help="use fram storage emulation")
group_sim.add_option("--disable-ekf2",
                     action='store_true',
                     help="disable EKF2 in build")
group_sim.add_option("--disable-ekf3",
                     action='store_true',
                     help="disable EKF3 in build")
group_sim.add_option("", "--start-time",
                     default=None,
                     type='string',
                     help="specify simulation start time in format YYYY-MM-DD-HH:MM in your local time zone")
group_sim.add_option("", "--sysid",
                     type='int',
                     default=None,
                     help="Set SYSID_THISMAV")
group_sim.add_option("--postype-single",
                     action='store_true',
                     help="force single precision postype_t")
group_sim.add_option("--ekf-double",
                     action='store_true',
                     help="use double precision in EKF")
group_sim.add_option("--ekf-single",
                     action='store_true',
                     help="use single precision in EKF")
group_sim.add_option("", "--slave",
                     type='int',
                     default=0,
                     help="Set the number of JSON slave")
group_sim.add_option("", "--auto-sysid",
                     default=False,
                     action='store_true',
                     help="Set SYSID_THISMAV based upon instance number")
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
group.add_option("", "--moddebug",
                 default=0,
                 type=int,
                 help="mavproxy module debug")
group.add_option("", "--no-rcin",
                 action='store_true',
                 help="disable mavproxy rcin")
parser.add_option_group(group)

group_completion = optparse.OptionGroup(parser, "Completion helpers")
group_completion.add_option("", "--list-vehicle",
                            action='store_true',
                            help="List the vehicles")
group_completion.add_option("", "--list-frame",
                            type='string',
                            default=None,
                            help="List the vehicle frames")
parser.add_option_group(group_completion)

cmd_opts, cmd_args = parser.parse_args()

if cmd_opts.list_vehicle:
    print(' '.join(vinfo.options.keys()))
    sys.exit(1)
if cmd_opts.list_frame:
    frame_options = sorted(vinfo.options[cmd_opts.list_frame]["frames"].keys())
    frame_options_string = ' '.join(frame_options)
    print(frame_options_string)
    sys.exit(1)

# clean up processes at exit:
atexit.register(kill_tasks)

progress("Start")

if cmd_opts.sim_vehicle_sh_compatible and cmd_opts.jobs is None:
    cmd_opts.jobs = 1

# validate parameters
if cmd_opts.valgrind and (cmd_opts.gdb or cmd_opts.gdb_stopped or cmd_opts.lldb or cmd_opts.lldb_stopped):
    print("May not use valgrind with gdb or lldb")
    sys.exit(1)

if cmd_opts.valgrind and cmd_opts.callgrind:
    print("May not use valgrind with callgrind")
    sys.exit(1)

if cmd_opts.strace and (cmd_opts.gdb or cmd_opts.gdb_stopped or cmd_opts.lldb or cmd_opts.lldb_stopped):
    print("May not use strace with gdb or lldb")
    sys.exit(1)

if (cmd_opts.gdb or cmd_opts.gdb_stopped) and (cmd_opts.lldb or cmd_opts.lldb_stopped):
    print("May not use lldb with gdb")
    sys.exit(1)

if cmd_opts.instance < 0:
    print("May not specify a negative instance ID")
    sys.exit(1)

if cmd_opts.count < 1:
    print("May not specify a count less than 1")
    sys.exit(1)

if cmd_opts.strace and cmd_opts.valgrind:
    print("valgrind and strace almost certainly not a good idea")

if cmd_opts.strace and cmd_opts.callgrind:
    print("callgrind and strace almost certainly not a good idea")

if cmd_opts.sysid and cmd_opts.auto_sysid:
    print("Cannot use auto-sysid together with sysid")
    sys.exit(1)

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

# map from some vehicle aliases back to canonical names.  APMrover2
# was the old name / directory name for Rover.
vehicle_map = {
    "APMrover2": "Rover",
    "Copter": "ArduCopter",  # will switch eventually
    "Plane": "ArduPlane",  # will switch eventually
    "Sub": "ArduSub",  # will switch eventually
    "copter": "ArduCopter",  # will switch eventually
    "plane": "ArduPlane",  # will switch eventually
    "sub": "ArduSub",  # will switch eventually
    "blimp" : "Blimp", # will switch eventually
}
if cmd_opts.vehicle in vehicle_map:
    progress("%s is now known as %s" %
             (cmd_opts.vehicle, vehicle_map[cmd_opts.vehicle]))
    cmd_opts.vehicle = vehicle_map[cmd_opts.vehicle]

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

frame_infos = vinfo.options_for_frame(cmd_opts.frame,
                                      cmd_opts.vehicle,
                                      cmd_opts)

vehicle_dir = os.path.realpath(os.path.join(root_dir, cmd_opts.vehicle))
if not os.path.exists(vehicle_dir):
    print("vehicle directory (%s) does not exist" % (vehicle_dir,))
    sys.exit(1)

if cmd_opts.instances is not None:
    instances = set()
    for i in cmd_opts.instances.split(' '):
        i = (int)(i)
        if i < 0:
            print("May not specify a negative instance ID")
            sys.exit(1)
        instances.add(i)
    instances = sorted(instances) # to list
else:
    instances = range(cmd_opts.instance, cmd_opts.instance + cmd_opts.count)

if cmd_opts.instance == 0:
    kill_tasks()

if cmd_opts.tracker:
    start_antenna_tracker(cmd_opts)

if cmd_opts.can_gps:
    start_CAN_GPS(cmd_opts)

if cmd_opts.custom_location:
    location = [(float)(x) for x in cmd_opts.custom_location.split(",")]
    progress("Starting up at %s" % (location,))
elif cmd_opts.location is not None:
    location = find_location_by_name(cmd_opts.location)
    progress("Starting up at %s (%s)" % (location, cmd_opts.location))
else:
    progress("Starting up at SITL location")
    location = None
if cmd_opts.swarm is not None:
    offsets = find_offsets(instances, cmd_opts.swarm)
elif cmd_opts.auto_offset_line is not None:
    if location is None:
        raise ValueError("location needed for auto-offset-line")
    (bearing, metres) = cmd_opts.auto_offset_line.split(",")
    bearing = float(bearing)
    metres = float(metres)
    dist = 0
    offsets = {}
    for x in instances:
        offsets[x] = [dist*math.sin(math.radians(bearing)), dist*math.cos(math.radians(bearing)), 0, 0]
        dist += metres
else:
    offsets = {x: [0.0, 0.0, 0.0, None] for x in instances}
if location is not None:
    spawns = find_spawns(location, offsets)
else:
    spawns = None

if cmd_opts.use_dir is not None:
    base_dir = os.path.realpath(cmd_opts.use_dir)
    try:
        os.makedirs(base_dir)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
    os.chdir(base_dir)
else:
    base_dir = os.getcwd()
instance_dir = []
if len(instances) == 1:
    instance_dir.append(base_dir)
else:
    for i in instances:
        i_dir = os.path.join(base_dir, str(i))
        try:
            os.makedirs(i_dir)
        except OSError as exception:
            if exception.errno != errno.EEXIST:
                raise
        finally:
            instance_dir.append(i_dir)

if True:
    if not cmd_opts.no_rebuild:  # i.e. we should rebuild
        do_build(cmd_opts, frame_infos)

    if cmd_opts.fresh_params:
        do_build_parameters(cmd_opts.vehicle)

    if cmd_opts.vehicle_binary is not None:
        vehicle_binary = cmd_opts.vehicle_binary
    elif cmd_opts.build_system == "waf":
        binary_basedir = "build/sitl"
        vehicle_binary = os.path.join(root_dir,
                                      binary_basedir,
                                      frame_infos["waf_target"])
    else:
        vehicle_binary = os.path.join(vehicle_dir, cmd_opts.vehicle + ".elf")

    if not os.path.exists(vehicle_binary):
        print("Vehicle binary (%s) does not exist" % (vehicle_binary,))
        sys.exit(1)

    start_vehicle(vehicle_binary,
                  cmd_opts,
                  frame_infos,
                  spawns=spawns)


if cmd_opts.delay_start:
    progress("Sleeping for %f seconds" % (cmd_opts.delay_start,))
    time.sleep(float(cmd_opts.delay_start))

tmp = None
if cmd_opts.frame in ['scrimmage-plane', 'scrimmage-copter']:
    # import only here so as to avoid jinja dependency in whole script
    from jinja2 import Environment, FileSystemLoader
    from tempfile import mkstemp
    entities = []
    config = {}
    config['plane'] = cmd_opts.vehicle == 'ArduPlane'
    if location is not None:
        config['lat'] = location[0]
        config['lon'] = location[1]
        config['alt'] = location[2]
    entities = {}
    for i in instances:
        (x, y, z, heading) = offsets[i]
        entities[i] = {
            'x': x, 'y': y, 'z': z, 'heading': heading,
            'to_ardupilot_port': 9003 + i * 10,
            'from_ardupilot_port': 9002 + i * 10,
            'to_ardupilot_ip': '127.0.0.1'
        }
    if cmd_opts.scrimmage_args is not None:
        scrimmage_args = cmd_opts.scrimmage_args.split(',')
        global_opts = ['terrain']
        instance_opts = ['motion_model', 'visual_model']
        for arg in scrimmage_args:
            arg = arg.split('=', 2)
            if len(arg) == 2:
                k, v = arg
                if k in global_opts:
                    config[k] = v
                elif k in instance_opts:
                    for i in entities:
                        # explicit instance args take precedence; don't overwrite
                        if k not in entities[i]:
                            entities[i][k] = v
            elif len(arg) == 3:
                i, k, v = arg
                try:
                    i = int(i)
                except ValueError:
                    continue
                if i in entities and k in instance_opts:
                    entities[i][k] = v
    config['entities'] = list(entities.values())
    env = Environment(loader=FileSystemLoader(os.path.join(autotest_dir, 'template')))
    mission = env.get_template('scrimmage.xml').render(**config)
    tmp = mkstemp()
    atexit.register(os.remove, tmp[1])

    with os.fdopen(tmp[0], 'w') as fd:
        fd.write(mission)
    run_in_terminal_window('SCRIMMAGE', ['scrimmage', tmp[1]])


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
