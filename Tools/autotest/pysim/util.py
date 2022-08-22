from __future__ import print_function

'''
AP_FLAKE8_CLEAN
'''

import atexit
import math
import os
import random
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from math import acos, atan2, cos, pi, sqrt

import pexpect

from pymavlink.rotmat import Vector3, Matrix3

if (sys.version_info[0] >= 3):
    ENCODING = 'ascii'
else:
    ENCODING = None

RADIUS_OF_EARTH = 6378100.0  # in meters

# List of open terminal windows for macosx
windowID = []


def m2ft(x):
    """Meters to feet."""
    return float(x) / 0.3048


def ft2m(x):
    """Feet to meters."""
    return float(x) * 0.3048


def kt2mps(x):
    return x * 0.514444444


def mps2kt(x):
    return x / 0.514444444


def topdir():
    """Return top of git tree where autotest is running from."""
    d = os.path.dirname(os.path.realpath(__file__))
    assert(os.path.basename(d) == 'pysim')
    d = os.path.dirname(d)
    assert(os.path.basename(d) == 'autotest')
    d = os.path.dirname(d)
    assert(os.path.basename(d) == 'Tools')
    d = os.path.dirname(d)
    return d


def relcurdir(path):
    """Return a path relative to current dir"""
    return os.path.relpath(path, os.getcwd())


def reltopdir(path):
    """Returns the normalized ABSOLUTE path for 'path', where path is a path relative to topdir"""
    return os.path.normpath(os.path.join(topdir(), path))


def run_cmd(cmd, directory=".", show=True, output=False, checkfail=True):
    """Run a shell command."""
    shell = False
    if not isinstance(cmd, list):
        cmd = [cmd]
        shell = True
    if show:
        print("Running: (%s) in (%s)" % (cmd_as_shell(cmd), directory,))
    if output:
        return subprocess.Popen(cmd, shell=shell, stdout=subprocess.PIPE, cwd=directory).communicate()[0]
    elif checkfail:
        return subprocess.check_call(cmd, shell=shell, cwd=directory)
    else:
        return subprocess.call(cmd, shell=shell, cwd=directory)


def rmfile(path):
    """Remove a file if it exists."""
    try:
        os.unlink(path)
    except Exception:
        pass


def deltree(path):
    """Delete a tree of files."""
    run_cmd('rm -rf %s' % path)


def relwaf():
    return "./modules/waf/waf-light"


def waf_configure(board,
                  j=None,
                  debug=False,
                  math_check_indexes=False,
                  coverage=False,
                  ekf_single=False,
                  postype_single=False,
                  sitl_32bit=False,
                  extra_args=[],
                  extra_hwdef=None,
                  ubsan=False,
                  ubsan_abort=False,
                  extra_defines={}):
    cmd_configure = [relwaf(), "configure", "--board", board]
    if debug:
        cmd_configure.append('--debug')
    if coverage:
        cmd_configure.append('--coverage')
    if math_check_indexes:
        cmd_configure.append('--enable-math-check-indexes')
    if ekf_single:
        cmd_configure.append('--ekf-single')
    if postype_single:
        cmd_configure.append('--postype-single')
    if sitl_32bit:
        cmd_configure.append('--sitl-32bit')
    if ubsan:
        cmd_configure.append('--ubsan')
    if ubsan_abort:
        cmd_configure.append('--ubsan-abort')
    if extra_hwdef is not None:
        cmd_configure.extend(['--extra-hwdef', extra_hwdef])
    for nv in extra_defines.items():
        cmd_configure.extend(['--define', "%s=%s" % nv])
    if j is not None:
        cmd_configure.extend(['-j', str(j)])
    pieces = [shlex.split(x) for x in extra_args]
    for piece in pieces:
        cmd_configure.extend(piece)
    run_cmd(cmd_configure, directory=topdir(), checkfail=True)


def waf_clean():
    run_cmd([relwaf(), "clean"], directory=topdir(), checkfail=True)


def waf_build(target=None):
    cmd = [relwaf(), "build"]
    if target is not None:
        cmd.append(target)
    run_cmd(cmd, directory=topdir(), checkfail=True)


def build_SITL(
        build_target,
        board='sitl',
        clean=True,
        configure=True,
        coverage=False,
        debug=False,
        ekf_single=False,
        extra_configure_args=[],
        extra_defines={},
        j=None,
        math_check_indexes=False,
        postype_single=False,
        sitl_32bit=False,
        ubsan=False,
        ubsan_abort=False,
):

    # first configure
    if configure:
        waf_configure(board,
                      j=j,
                      debug=debug,
                      math_check_indexes=math_check_indexes,
                      ekf_single=ekf_single,
                      postype_single=postype_single,
                      coverage=coverage,
                      sitl_32bit=sitl_32bit,
                      ubsan=ubsan,
                      ubsan_abort=ubsan_abort,
                      extra_defines=extra_defines,
                      extra_args=extra_configure_args)

    # then clean
    if clean:
        waf_clean()

    # then build
    cmd_make = [relwaf(), "build", "--target", build_target]
    if j is not None:
        cmd_make.extend(['-j', str(j)])
    run_cmd(cmd_make, directory=topdir(), checkfail=True, show=True)
    return True


def build_examples(board, j=None, debug=False, clean=False, configure=True, math_check_indexes=False, coverage=False,
                   ekf_single=False, postype_single=False, sitl_32bit=False, ubsan=False, ubsan_abort=False,
                   extra_configure_args=[]):
    # first configure
    if configure:
        waf_configure(board,
                      j=j,
                      debug=debug,
                      math_check_indexes=math_check_indexes,
                      ekf_single=ekf_single,
                      postype_single=postype_single,
                      coverage=coverage,
                      sitl_32bit=sitl_32bit,
                      ubsan=ubsan,
                      ubsan_abort=ubsan_abort,
                      extra_args=extra_configure_args)

    # then clean
    if clean:
        waf_clean()

    # then build
    cmd_make = [relwaf(), "examples"]
    run_cmd(cmd_make, directory=topdir(), checkfail=True, show=True)
    return True


def build_replay(board, j=None, debug=False, clean=False):
    # first configure
    waf_configure(board, j=j, debug=debug)

    # then clean
    if clean:
        waf_clean()

    # then build
    cmd_make = [relwaf(), "replay"]
    run_cmd(cmd_make, directory=topdir(), checkfail=True, show=True)
    return True


def build_tests(board,
                j=None,
                debug=False,
                clean=False,
                configure=True,
                math_check_indexes=False,
                coverage=False,
                ekf_single=False,
                postype_single=False,
                sitl_32bit=False,
                ubsan=False,
                ubsan_abort=False,
                extra_configure_args=[]):

    # first configure
    if configure:
        waf_configure(board,
                      j=j,
                      debug=debug,
                      math_check_indexes=math_check_indexes,
                      ekf_single=ekf_single,
                      postype_single=postype_single,
                      coverage=coverage,
                      sitl_32bit=sitl_32bit,
                      ubsan=ubsan,
                      ubsan_abort=ubsan_abort,
                      extra_args=extra_configure_args)

    # then clean
    if clean:
        waf_clean()

    # then build
    run_cmd([relwaf(), "tests"], directory=topdir(), checkfail=True, show=True)
    return True


# list of pexpect children to close on exit
close_list = []


def pexpect_autoclose(p):
    """Mark for autoclosing."""
    global close_list
    close_list.append(p)


def pexpect_close(p):
    """Close a pexpect child."""
    global close_list

    ex = None
    if p is None:
        print("Nothing to close")
        return
    try:
        p.kill(signal.SIGTERM)
    except IOError as e:
        print("Caught exception: %s" % str(e))
        ex = e
        pass
    if ex is None:
        # give the process some time to go away
        for i in range(20):
            if not p.isalive():
                break
            time.sleep(0.05)
    try:
        p.close()
    except Exception:
        pass
    try:
        p.close(force=True)
    except Exception:
        pass
    if p in close_list:
        close_list.remove(p)


def pexpect_close_all():
    """Close all pexpect children."""
    global close_list
    for p in close_list[:]:
        pexpect_close(p)


def pexpect_drain(p):
    """Drain any pending input."""
    try:
        p.read_nonblocking(1000, timeout=0)
    except Exception:
        pass


def cmd_as_shell(cmd):
    return (" ".join(['"%s"' % x for x in cmd]))


def make_safe_filename(text):
    """Return a version of text safe for use as a filename."""
    r = re.compile("([^a-zA-Z0-9_.+-])")
    text.replace('/', '-')
    filename = r.sub(lambda m: str(hex(ord(str(m.group(1))))).upper(), text)
    return filename


def valgrind_log_filepath(binary, model):
    return make_safe_filename('%s-%s-valgrind.log' % (os.path.basename(binary), model,))


def kill_screen_gdb():
    cmd = ["screen", "-X", "-S", "ardupilot-gdb", "quit"]
    subprocess.Popen(cmd)


def kill_mac_terminal():
    global windowID
    for window in windowID:
        cmd = ("osascript -e \'tell application \"Terminal\" to close "
               "(window(get index of window id %s))\'" % window)
        os.system(cmd)


class FakeMacOSXSpawn(object):
    '''something that looks like a pspawn child so we can ignore attempts
    to pause (and otherwise kill(1) SITL.  MacOSX using osascript to
    start/stop sitl
    '''
    def __init__(self):
        pass

    def progress(self, message):
        print(message)

    def kill(self, sig):
        # self.progress("FakeMacOSXSpawn: ignoring kill(%s)" % str(sig))
        pass

    def isalive(self):
        self.progress("FakeMacOSXSpawn: assuming process is alive")
        return True


def start_SITL(binary,
               valgrind=False,
               callgrind=False,
               gdb=False,
               gdb_no_tui=False,
               wipe=False,
               synthetic_clock=True,
               home=None,
               model=None,
               speedup=1,
               defaults_filepath=None,
               unhide_parameters=False,
               gdbserver=False,
               breakpoints=[],
               disable_breakpoints=False,
               customisations=[],
               lldb=False,
               enable_fgview_output=False,
               supplementary=False):

    if model is None and not supplementary:
        raise ValueError("model must not be None")

    """Launch a SITL instance."""
    cmd = []
    if (callgrind or valgrind) and os.path.exists('/usr/bin/valgrind'):
        # we specify a prefix for vgdb-pipe because on Vagrant virtual
        # machines the pipes are created on the mountpoint for the
        # shared directory with the host machine.  mmap's,
        # unsurprisingly, fail on files created on that mountpoint.
        vgdb_prefix = os.path.join(tempfile.gettempdir(), "vgdb-pipe")
        log_file = valgrind_log_filepath(binary=binary, model=model)
        cmd.extend([
            'valgrind',
            # adding this option allows valgrind to cope with the overload
            # of operator new
            "--soname-synonyms=somalloc=nouserintercepts",
            '--vgdb-prefix=%s' % vgdb_prefix,
            '-q',
            '--log-file=%s' % log_file])
        if callgrind:
            cmd.extend(["--tool=callgrind"])
    if gdbserver:
        cmd.extend(['gdbserver', 'localhost:3333'])
        if gdb:
            # attach gdb to the gdbserver:
            f = open("/tmp/x.gdb", "w")
            f.write("target extended-remote localhost:3333\nc\n")
            for breakpoint in breakpoints:
                f.write("b %s\n" % (breakpoint,))
            if disable_breakpoints:
                f.write("disable\n")
            f.close()
            run_cmd('screen -d -m -S ardupilot-gdbserver '
                    'bash -c "gdb -x /tmp/x.gdb"')
    elif gdb:
        f = open("/tmp/x.gdb", "w")
        f.write("set pagination off\n")
        for breakpoint in breakpoints:
            f.write("b %s\n" % (breakpoint,))
        if disable_breakpoints:
            f.write("disable\n")
        if not gdb_no_tui:
            f.write("tui enable\n")
        f.write("r\n")
        f.close()
        if sys.platform == "darwin" and os.getenv('DISPLAY'):
            cmd.extend(['gdb', '-x', '/tmp/x.gdb', '--args'])
        elif os.environ.get('DISPLAY'):
            cmd.extend(['xterm', '-e', 'gdb', '-x', '/tmp/x.gdb', '--args'])
        else:
            cmd.extend(['screen',
                        '-L', '-Logfile', 'gdb.log',
                        '-d',
                        '-m',
                        '-S', 'ardupilot-gdb',
                        'gdb', '-x', '/tmp/x.gdb', binary, '--args'])
    elif lldb:
        f = open("/tmp/x.lldb", "w")
        for breakpoint in breakpoints:
            f.write("b %s\n" % (breakpoint,))
        if disable_breakpoints:
            f.write("disable\n")
        f.write("settings set target.process.stop-on-exec false\n")
        f.write("process launch\n")
        f.close()
        if sys.platform == "darwin" and os.getenv('DISPLAY'):
            cmd.extend(['lldb', '-s', '/tmp/x.lldb', '--'])
        elif os.environ.get('DISPLAY'):
            cmd.extend(['xterm', '-e', 'lldb', '-s', '/tmp/x.lldb', '--'])
        else:
            raise RuntimeError("DISPLAY was not set")

    cmd.append(binary)
    if not supplementary:
        if wipe:
            cmd.append('-w')
        if synthetic_clock:
            cmd.append('-S')
        if home is not None:
            cmd.extend(['--home', home])
        cmd.extend(['--model', model])
        if speedup != 1:
            cmd.extend(['--speedup', str(speedup)])
        if defaults_filepath is not None:
            if type(defaults_filepath) == list:
                defaults = [reltopdir(path) for path in defaults_filepath]
                if len(defaults):
                    cmd.extend(['--defaults', ",".join(defaults)])
            else:
                cmd.extend(['--defaults', reltopdir(defaults_filepath)])
        if unhide_parameters:
            cmd.extend(['--unhide-groups'])
        # somewhere for MAVProxy to connect to:
        cmd.append('--uartC=tcp:2')
        if not enable_fgview_output:
            cmd.append("--disable-fgview")

    cmd.extend(customisations)

    if (gdb or lldb) and sys.platform == "darwin" and os.getenv('DISPLAY'):
        global windowID
        # on MacOS record the window IDs so we can close them later
        atexit.register(kill_mac_terminal)
        child = None
        mydir = os.path.dirname(os.path.realpath(__file__))
        autotest_dir = os.path.realpath(os.path.join(mydir, '..'))
        runme = [os.path.join(autotest_dir, "run_in_terminal_window.sh"), 'mactest']
        runme.extend(cmd)
        print(cmd)
        out = subprocess.Popen(runme, stdout=subprocess.PIPE).communicate()[0]
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
            print("Cannot find %s process terminal" % binary)
        child = FakeMacOSXSpawn()
    elif gdb and not os.getenv('DISPLAY'):
        subprocess.Popen(cmd)
        atexit.register(kill_screen_gdb)
        # we are expected to return a pexpect wrapped around the
        # stdout of the ArduPilot binary.  Not going to happen until
        # AP gets a redirect-stdout-to-filehandle option.  So, in the
        # meantime, return a dummy:
        return pexpect.spawn("true", ["true"],
                             logfile=sys.stdout,
                             encoding=ENCODING,
                             timeout=5)
    else:
        print("Running: %s" % cmd_as_shell(cmd))

        first = cmd[0]
        rest = cmd[1:]
        child = pexpect.spawn(first, rest, logfile=sys.stdout, encoding=ENCODING, timeout=5)
        pexpect_autoclose(child)
    # give time for parameters to properly setup
    time.sleep(3)
    if gdb or lldb:
        # if we run GDB we do so in an xterm.  "Waiting for
        # connection" is never going to appear on xterm's output.
        # ... so let's give it another magic second.
        time.sleep(1)
        # TODO: have a SITL-compiled ardupilot able to have its
        # console on an output fd.
    else:
        child.expect('Waiting for ', timeout=300)
    return child


def mavproxy_cmd():
    '''return path to which mavproxy to use'''
    return os.getenv('MAVPROXY_CMD', 'mavproxy.py')


def MAVProxy_version():
    '''return the current version of mavproxy as a tuple e.g. (1,8,8)'''
    command = "%s --version" % mavproxy_cmd()
    output = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE).communicate()[0]
    output = output.decode('ascii')
    match = re.search("MAVProxy Version: ([0-9]+)[.]([0-9]+)[.]([0-9]+)", output)
    if match is None:
        raise ValueError("Unable to determine MAVProxy version from (%s)" % output)
    return (int(match.group(1)), int(match.group(2)), int(match.group(3)))


def start_MAVProxy_SITL(atype,
                        aircraft=None,
                        setup=False,
                        master='tcp:127.0.0.1:5762',
                        options=[],
                        pexpect_timeout=60,
                        logfile=sys.stdout):
    """Launch mavproxy connected to a SITL instance."""
    local_mp_modules_dir = os.path.abspath(
        os.path.join(__file__, '..', '..', '..', 'mavproxy_modules'))
    env = dict(os.environ)
    old = env.get('PYTHONPATH', None)
    env['PYTHONPATH'] = local_mp_modules_dir
    if old is not None:
        env['PYTHONPATH'] += os.path.pathsep + old

    import pexpect
    global close_list
    cmd = []
    cmd.append(mavproxy_cmd())
    cmd.extend(['--master', master])
    if setup:
        cmd.append('--setup')
    if aircraft is None:
        aircraft = 'test.%s' % atype
    cmd.extend(['--aircraft', aircraft])
    cmd.extend(options)
    cmd.extend(['--default-modules', 'misc,wp,rally,fence,param,arm,mode,rc,cmdlong,output'])

    print("PYTHONPATH: %s" % str(env['PYTHONPATH']))
    print("Running: %s" % cmd_as_shell(cmd))

    ret = pexpect.spawn(cmd[0], cmd[1:], logfile=logfile, encoding=ENCODING, timeout=pexpect_timeout, env=env)
    ret.delaybeforesend = 0
    pexpect_autoclose(ret)
    return ret


def expect_setup_callback(e, callback):
    """Setup a callback that is called once a second while waiting for
       patterns."""
    import pexpect

    def _expect_callback(pattern, timeout=e.timeout):
        tstart = time.time()
        while time.time() < tstart + timeout:
            try:
                ret = e.expect_saved(pattern, timeout=1)
                return ret
            except pexpect.TIMEOUT:
                e.expect_user_callback(e)
        print("Timed out looking for %s" % pattern)
        raise pexpect.TIMEOUT(timeout)

    e.expect_user_callback = callback
    e.expect_saved = e.expect
    e.expect = _expect_callback


def mkdir_p(directory):
    """Like mkdir -p ."""
    if not directory:
        return
    if directory.endswith("/"):
        mkdir_p(directory[:-1])
        return
    if os.path.isdir(directory):
        return
    mkdir_p(os.path.dirname(directory))
    os.mkdir(directory)


def loadfile(fname):
    """Load a file as a string."""
    f = open(fname, mode='r')
    r = f.read()
    f.close()
    return r


def lock_file(fname):
    """Lock a file."""
    import fcntl
    f = open(fname, mode='w')
    try:
        fcntl.lockf(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        return None
    return f


def check_parent(parent_pid=None):
    """Check our parent process is still alive."""
    if parent_pid is None:
        try:
            parent_pid = os.getppid()
        except Exception:
            pass
    if parent_pid is None:
        return
    try:
        os.kill(parent_pid, 0)
    except Exception:
        print("Parent had finished - exiting")
        sys.exit(1)


def EarthRatesToBodyRates(dcm, earth_rates):
    """Convert the angular velocities from earth frame to
    body frame. Thanks to James Goppert for the formula

    all inputs and outputs are in radians

    returns a gyro vector in body frame, in rad/s .
    """
    from math import sin, cos

    (phi, theta, psi) = dcm.to_euler()
    phiDot   = earth_rates.x
    thetaDot = earth_rates.y
    psiDot   = earth_rates.z

    p = phiDot - psiDot * sin(theta)
    q = cos(phi) * thetaDot + sin(phi) * psiDot * cos(theta)
    r = cos(phi) * psiDot * cos(theta) - sin(phi) * thetaDot
    return Vector3(p, q, r)


def BodyRatesToEarthRates(dcm, gyro):
    """Convert the angular velocities from body frame to
    earth frame.

    all inputs and outputs are in radians/s

    returns a earth rate vector.
    """
    from math import sin, cos, tan, fabs

    p      = gyro.x
    q      = gyro.y
    r      = gyro.z

    (phi, theta, psi) = dcm.to_euler()

    phiDot   = p + tan(theta) * (q * sin(phi) + r * cos(phi))
    thetaDot = q * cos(phi) - r * sin(phi)
    if fabs(cos(theta)) < 1.0e-20:
        theta += 1.0e-10
    psiDot   = (q * sin(phi) + r * cos(phi)) / cos(theta)
    return Vector3(phiDot, thetaDot, psiDot)


def gps_newpos(lat, lon, bearing, distance):
    """Extrapolate latitude/longitude given a heading and distance
    thanks to http://www.movable-type.co.uk/scripts/latlong.html .
    """
    from math import sin, asin, cos, atan2, radians, degrees

    lat1 = radians(lat)
    lon1 = radians(lon)
    brng = radians(bearing)
    dr = distance / RADIUS_OF_EARTH

    lat2 = asin(sin(lat1) * cos(dr) +
                cos(lat1) * sin(dr) * cos(brng))
    lon2 = lon1 + atan2(sin(brng) * sin(dr) * cos(lat1),
                        cos(dr) - sin(lat1) * sin(lat2))
    return (degrees(lat2), degrees(lon2))


def gps_distance(lat1, lon1, lat2, lon2):
    """Return distance between two points in meters,
    coordinates are in degrees
    thanks to http://www.movable-type.co.uk/scripts/latlong.html ."""
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = math.sin(0.5 * dLat)**2 + math.sin(0.5 * dLon)**2 * math.cos(lat1) * math.cos(lat2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return RADIUS_OF_EARTH * c


def gps_bearing(lat1, lon1, lat2, lon2):
    """Return bearing between two points in degrees, in range 0-360
    thanks to http://www.movable-type.co.uk/scripts/latlong.html ."""
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    bearing = math.degrees(math.atan2(y, x))
    if bearing < 0:
        bearing += 360.0
    return bearing


class Wind(object):
    """A wind generation object."""
    def __init__(self, windstring, cross_section=0.1):
        a = windstring.split(',')
        if len(a) != 3:
            raise RuntimeError("Expected wind in speed,direction,turbulance form, not %s" % windstring)
        self.speed      = float(a[0])  # m/s
        self.direction  = float(a[1])  # direction the wind is going in
        self.turbulance = float(a[2])  # turbulance factor (standard deviation)

        # the cross-section of the aircraft to wind. This is multiplied by the
        # difference in the wind and the velocity of the aircraft to give the acceleration
        self.cross_section = cross_section

        # the time constant for the turbulance - the average period of the
        # changes over time
        self.turbulance_time_constant = 5.0

        # wind time record
        self.tlast = time.time()

        # initial turbulance multiplier
        self.turbulance_mul = 1.0

    def current(self, deltat=None):
        """Return current wind speed and direction as a tuple
        speed is in m/s, direction in degrees."""
        if deltat is None:
            tnow = time.time()
            deltat = tnow - self.tlast
            self.tlast = tnow

        # update turbulance random walk
        w_delta = math.sqrt(deltat) * (1.0 - random.gauss(1.0, self.turbulance))
        w_delta -= (self.turbulance_mul - 1.0) * (deltat / self.turbulance_time_constant)
        self.turbulance_mul += w_delta
        speed = self.speed * math.fabs(self.turbulance_mul)
        return (speed, self.direction)

    # Calculate drag.
    def drag(self, velocity, deltat=None):
        """Return current wind force in Earth frame.  The velocity parameter is
           a Vector3 of the current velocity of the aircraft in earth frame, m/s ."""
        from math import radians

        # (m/s, degrees) : wind vector as a magnitude and angle.
        (speed, direction) = self.current(deltat=deltat)
        # speed = self.speed
        # direction = self.direction

        # Get the wind vector.
        w = toVec(speed, radians(direction))

        obj_speed = velocity.length()

        # Compute the angle between the object vector and wind vector by taking
        # the dot product and dividing by the magnitudes.
        d = w.length() * obj_speed
        if d == 0:
            alpha = 0
        else:
            alpha = acos((w * velocity) / d)

        # Get the relative wind speed and angle from the object.  Note that the
        # relative wind speed includes the velocity of the object; i.e., there
        # is a headwind equivalent to the object's speed even if there is no
        # absolute wind.
        (rel_speed, beta) = apparent_wind(speed, obj_speed, alpha)

        # Return the vector of the relative wind, relative to the coordinate
        # system.
        relWindVec = toVec(rel_speed, beta + atan2(velocity.y, velocity.x))

        # Combine them to get the acceleration vector.
        return Vector3(acc(relWindVec.x, drag_force(self, relWindVec.x)), acc(relWindVec.y, drag_force(self, relWindVec.y)), 0)


def apparent_wind(wind_sp, obj_speed, alpha):
    """http://en.wikipedia.org/wiki/Apparent_wind

    Returns apparent wind speed and angle of apparent wind.  Alpha is the angle
    between the object and the true wind.  alpha of 0 rads is a headwind; pi a
    tailwind.  Speeds should always be positive."""
    delta = wind_sp * cos(alpha)
    x = wind_sp**2 + obj_speed**2 + 2 * obj_speed * delta
    rel_speed = sqrt(x)
    if rel_speed == 0:
        beta = pi
    else:
        beta = acos((delta + obj_speed) / rel_speed)

    return (rel_speed, beta)


def drag_force(wind, sp):
    """See http://en.wikipedia.org/wiki/Drag_equation

    Drag equation is F(a) = cl * p/2 * v^2 * a, where cl : drag coefficient
    (let's assume it's low, .e.g., 0.2), p : density of air (assume about 1
    kg/m^3, the density just over 1500m elevation), v : relative speed of wind
    (to the body), a : area acted on (this is captured by the cross_section
    parameter).

    So then we have
    F(a) = 0.2 * 1/2 * v^2 * cross_section = 0.1 * v^2 * cross_section."""
    return (sp**2.0) * 0.1 * wind.cross_section


def acc(val, mag):
    """ Function to make the force vector.  relWindVec is the direction the apparent
    wind comes *from*.  We want to compute the accleration vector in the direction
    the wind blows to."""
    if val == 0:
        return mag
    else:
        return (val / abs(val)) * (0 - mag)


def toVec(magnitude, angle):
    """Converts a magnitude and angle (radians) to a vector in the xy plane."""
    v = Vector3(magnitude, 0, 0)
    m = Matrix3()
    m.from_euler(0, 0, angle)
    return m.transposed() * v


def constrain(value, minv, maxv):
    """Constrain a value to a range."""
    if value < minv:
        value = minv
    if value > maxv:
        value = maxv
    return value


def load_local_module(fname):
    '''load a python module from within the ardupilot tree'''
    fname = os.path.join(topdir(), fname)
    if sys.version_info.major >= 3:
        import importlib.util
        spec = importlib.util.spec_from_file_location("local_module", fname)
        ret = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ret)
    else:
        import imp
        ret = imp.load_source("local_module", fname)
    return ret


if __name__ == "__main__":
    import doctest
    doctest.testmod()
