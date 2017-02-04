from __future__ import print_function
import math
import os
import random
import re
import sys
import time
from math import acos, atan2, cos, pi, sqrt
from subprocess import PIPE, Popen, call, check_call

import pexpect

from . rotmat import Matrix3, Vector3

if (sys.version_info[0] >= 3):
    ENCODING = 'ascii'
else:
    ENCODING = None

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


def reltopdir(path):
    """Return a path relative to topdir()."""
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
        return Popen(cmd, shell=shell, stdout=PIPE, cwd=directory).communicate()[0]
    elif checkfail:
        return check_call(cmd, shell=shell, cwd=directory)
    else:
        return call(cmd, shell=shell, cwd=directory)


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


def waf_configure(board, j=None, debug=False):
    cmd_configure = [relwaf(), "configure", "--board", board]
    if debug:
        cmd_configure.append('--debug')
    if j is not None:
        cmd_configure.extend(['-j', str(j)])
    run_cmd(cmd_configure, directory=topdir(), checkfail=True)


def waf_clean():
    run_cmd([relwaf(), "clean"], directory=topdir(), checkfail=True)


def build_SITL(build_target, j=None, debug=False, board='sitl'):
    """Build desktop SITL."""

    # first configure
    waf_configure(board, j=j, debug=debug)

    # then clean
    waf_clean()

    # then build
    cmd_make = [relwaf(), "build", "--target", build_target]
    if j is not None:
        cmd_make.extend(['-j', str(j)])
    run_cmd(cmd_make, directory=topdir(), checkfail=True, show=True)
    return True


def build_examples(board, j=None, debug=False, clean=False):
    # first configure
    waf_configure(board, j=j, debug=debug)

    # then clean
    if clean:
        waf_clean()

    # then build
    cmd_make = [relwaf(), "examples"]
    run_cmd(cmd_make, directory=topdir(), checkfail=True, show=True)
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
    import pexpect
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
    filename = r.sub(lambda m: "%" + str(hex(ord(str(m.group(1))))).upper(), text)
    return filename


def valgrind_log_filepath(binary, model):
    return make_safe_filename('%s-%s-valgrind.log' % (os.path.basename(binary), model,))


def start_SITL(binary, valgrind=False, gdb=False, wipe=False, synthetic_clock=True, home=None, model=None, speedup=1, defaults_file=None, unhide_parameters=False):
    """Launch a SITL instance."""
    cmd = []
    if valgrind and os.path.exists('/usr/bin/valgrind'):
        cmd.extend(['valgrind', '-q', '--log-file=%s' % valgrind_log_filepath(binary=binary, model=model)])
    if gdb:
        f = open("/tmp/x.gdb", "w")
        f.write("r\n")
        f.close()
        cmd.extend(['xterm', '-e', 'gdb', '-x', '/tmp/x.gdb', '--args'])
    
    cmd.append(binary)
    if wipe:
        cmd.append('-w')
    if synthetic_clock:
        cmd.append('-S')
    if home is not None:
        cmd.extend(['--home', home])
    if model is not None:
        cmd.extend(['--model', model])
    if speedup != 1:
        cmd.extend(['--speedup', str(speedup)])
    if defaults_file is not None:
        cmd.extend(['--defaults', defaults_file])
    if unhide_parameters:
        cmd.extend(['--unhide-groups'])
    print("Running: %s" % cmd_as_shell(cmd))
    first = cmd[0]
    rest = cmd[1:]
    child = pexpect.spawn(first, rest, logfile=sys.stdout, encoding=ENCODING, timeout=5)
    delaybeforesend = 0
    pexpect_autoclose(child)
    # give time for parameters to properly setup
    time.sleep(3)
    if gdb:
        # if we run GDB we do so in an xterm.  "Waiting for
        # connection" is never going to appear on xterm's output.
        # ... so let's give it another magic second.
        time.sleep(1)
        # TODO: have a SITL-compiled ardupilot able to have its
        # console on an output fd.
    else:
        child.expect('Waiting for connection', timeout=300)
    return child


def start_MAVProxy_SITL(atype, aircraft=None, setup=False, master='tcp:127.0.0.1:5760',
                        options=None, logfile=sys.stdout):
    """Launch mavproxy connected to a SITL instance."""
    import pexpect
    global close_list
    MAVPROXY = os.getenv('MAVPROXY_CMD', 'mavproxy.py')
    cmd = MAVPROXY + ' --master=%s --out=127.0.0.1:14550' % master
    if setup:
        cmd += ' --setup'
    if aircraft is None:
        aircraft = 'test.%s' % atype
    cmd += ' --aircraft=%s' % aircraft
    if options is not None:
        cmd += ' ' + options
    ret = pexpect.spawn(cmd, logfile=logfile, encoding=ENCODING, timeout=60)
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
                pass
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

radius_of_earth = 6378100.0  # in meters


def gps_newpos(lat, lon, bearing, distance):
    """Extrapolate latitude/longitude given a heading and distance
    thanks to http://www.movable-type.co.uk/scripts/latlong.html .
    """
    from math import sin, asin, cos, atan2, radians, degrees

    lat1 = radians(lat)
    lon1 = radians(lon)
    brng = radians(bearing)
    dr = distance / radius_of_earth

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
    return radius_of_earth * c


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
    def drag(self, velocity, deltat=None, testing=None):
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

if __name__ == "__main__":
    import doctest
    doctest.testmod()
