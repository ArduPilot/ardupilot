#!/usr/bin/env python3
'''
Multi-vehicle cluster test: copters follow a plane.

Launches several real SITL processes - one ArduPlane and two ArduCopter -
into a single shared-memory cluster (--cluster=ID), flies the plane in a
loiter, and puts the copters into FOLLOW mode behind it.

Why this lives outside vehicle_test_suite.py: that harness is built around
a single self.mav connection (change_mode/takeoff/wait_altitude/context all
target one vehicle), and its only multi-binary hook, sup_binaries, is
populated solely from autotest.py --sup-binary and gives the extra
processes no MAVLink plumbing. Flying several vehicles needs independent
connections, so this drives pymavlink directly.

Separate SITL processes share a clock through the cluster but not a MAVLink
bus, so this script also acts as the telemetry mesh: it forwards the plane's
GLOBAL_POSITION_INT onto each copter's link, preserving the plane's sysid in
the header, which is what AP_Follow matches against FOLL_SYSID.

Everything is measured in SIMULATED time
----------------------------------------
The requested speedup is 100x, but a loaded machine - CI especially - will
not achieve it, and the achieved rate varies during a run. So every quantity
that depends on what the vehicles DO is expressed in simulated seconds, read
straight out of the cluster segment: phase timeouts, how long a copter has
held station, how stale the copters' view of the plane is, and the arm/mode
waits that are really waiting on EKF convergence. A slower host then flies
exactly the same flight, just taking longer in wall time, instead of quietly
giving the copters less time to converge and flaking.

Only three things are deliberately in wall time, because they do not scale
with the simulation: process startup, the bridge loop period (it is a real
loop on a real CPU - its effect is measured in sim time), the console report
interval (a human reading rate), and the backstop that stops a wedged run
from hanging forever.

The achieved speedup is measured and printed continuously, which is usually
what you want to know when this runs slowly in CI.

The test asserts:
  1. the cluster holds - every vehicle stays registered and their
     simulated clocks stay within a bounded skew of each other
  2. follow works - each copter closes on the plane and holds station

AP_FLAKE8_CLEAN
'''

import glob
import math
import mmap
import optparse
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil

# layout of AP_SITL_ShmData, see libraries/AP_HAL_SITL/SITL_SharedMem.h
SHM_HEADER_LEN = 24                       # magic, version, slave epoch (wall, sim)
SHM_SLOT_LEN = 8 + 4 + 4 + 40 + 4096      # sim_time, pid, armed, mutex, payload
SHM_MAGIC = 0x41505351                    # "APSQ", v5: shared slave epoch

HOME = "-35.363261,149.165230,584,353"

# AP_Follow discards its target after this many seconds of SIM time without
# an update. The default is 3s, which a wall-time bridge cannot honour at
# high speedup (at 100x even a 100Hz bridge is one update per sim-second),
# so the copters are told to tolerate a longer gap. AP_Follow extrapolates
# the target from its reported velocity between updates, so a wider window
# costs tracking accuracy rather than breaking follow outright.
FOLL_TIMEOUT_S = 30.0


class TestFailure(Exception):
    pass


def shm_path(cluster_id):
    return "/dev/shm/ardupilot_sitl_cluster_%d_%d" % (os.getuid(), cluster_id)


def set_cpu_governor_performance():
    """best effort: hold the CPU clocks up for the duration of the test

    Measured on an i7 laptop: three busy cores drop a lone SITL vehicle
    from 26x to 8x purely through frequency scaling, which dwarfs every
    software cost in the cluster. Writing the governor needs root; CI
    containers run privileged so it works there, and on a developer
    machine a failure is only reported, never fatal.

    Returns [(path, previous_governor)] for restore_cpu_governors().
    """
    saved = []
    import glob
    paths = sorted(glob.glob(
        "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor"))
    if not paths:
        return saved
    denied = False
    for path in paths:
        try:
            with open(path) as f:
                old = f.read().strip()
            if old == "performance":
                continue
            with open(path, "w") as f:
                f.write("performance")
            saved.append((path, old))
        except OSError:
            denied = True
    # pin the floor to the ceiling as well: the performance governor
    # alone still lets some platforms sag under multi-core load
    for gov_path, _ in list(saved):
        base = os.path.dirname(gov_path)
        try:
            with open(os.path.join(base, "cpuinfo_max_freq")) as f:
                fmax = f.read().strip()
            minp = os.path.join(base, "scaling_min_freq")
            with open(minp) as f:
                old_min = f.read().strip()
            if old_min != fmax:
                with open(minp, "w") as f:
                    f.write(fmax)
                saved.append((minp, old_min))
        except OSError:
            pass
    if saved:
        progress("CPU governor -> performance on %u cores" % len(saved))
    if denied and not saved:
        progress("NOTE: could not set the CPU governor to performance "
                 "(needs root); expect lower and noisier speedups - "
                 "sudo cpupower frequency-set -g performance")
    return saved


def restore_cpu_governors(saved):
    for path, old in saved:
        try:
            with open(path, "w") as f:
                f.write(old)
        except OSError:
            pass
    if saved:
        progress("CPU governors restored")


def progress(text):
    print("CLUSTER-FOLLOW: %s" % text)


def cpu_temp_report():
    '''hottest CPU thermal zone from sysfs, e.g. "cpu temp 100C
    (x86_pkg_temp)"; None when unavailable. Printed periodically so every
    log records whether the silicon was thermally throttled: at 100C the
    package hard-caps its clocks and no speedup number is meaningful.'''
    zones = []
    for z in glob.glob('/sys/class/thermal/thermal_zone*'):
        try:
            with open(os.path.join(z, 'temp')) as f:
                t = int(f.read().strip()) / 1000.0
            with open(os.path.join(z, 'type')) as f:
                ty = f.read().strip()
        except (OSError, ValueError):
            continue
        zones.append((t, ty))
    if not zones:
        return None
    t, ty = max(zones)
    return "cpu temp %.0fC (%s)" % (t, ty)
    sys.stdout.flush()


class ClusterView(object):
    '''live read-only view of the cluster segment

    mmap'd rather than re-read each time: this is sampled every time round
    a fast bridge loop, and re-opening a 64KB file at that rate is wasteful.
    '''

    def __init__(self, cluster_id, instances):
        self.cluster_id = cluster_id
        self.instances = instances
        self.f = open(shm_path(cluster_id), "rb")
        self.mm = mmap.mmap(self.f.fileno(), 0, access=mmap.ACCESS_READ)
        magic, = struct.unpack_from("<I", self.mm, 0)
        if magic != SHM_MAGIC:
            raise TestFailure("cluster segment has bad magic 0x%08X" % magic)

    last_time = None

    def members(self):
        if self.last_time is None:
            self.last_time = {}
        '''{instance: sim_time_us} for slots with a live pid'''
        out = {}
        for i in self.instances:
            # the vehicles write these 64-bit times concurrently and
            # python reads the mmap byte-wise, so a single read can
            # TEAR mid-carry and produce a +-2^32us phantom (+-71
            # minutes) - measured as impossible "5187 sim-seconds" in
            # a 600s phase. Read until two consecutive reads agree,
            # and never let a member's time step backwards.
            off = SHM_HEADER_LEN + i * SHM_SLOT_LEN
            sim_time, pid = struct.unpack_from("<qi", self.mm, off)
            for _ in range(4):
                again, pid2 = struct.unpack_from("<qi", self.mm, off)
                if again == sim_time:
                    break
                sim_time, pid = again, pid2
            if pid > 0:
                prev = self.last_time.get(i)
                if prev is not None and sim_time < prev:
                    sim_time = prev
                self.last_time[i] = sim_time
                out[i] = sim_time
        return out

    def now(self):
        '''current simulated time in seconds, or None if nobody is present

        Deliberately the MOST advanced member, not the least. A vehicle
        whose clock has frozen - blocked on a full socket, say - would
        otherwise pin this value and silently disable every sim deadline
        in the test, turning a clean timeout into a hang. Divergence
        between members is caught by the skew check instead.
        '''
        members = self.members()
        if not members:
            return None
        return max(members.values()) * 1e-6

    def close(self):
        try:
            self.mm.close()
            self.f.close()
        except (OSError, ValueError):
            pass


class Deadline(object):
    '''a timeout measured in SIM seconds, with a wall-clock backstop

    The sim limit is the real deadline - it makes a phase take the same
    amount of simulated flying regardless of how fast the host runs. The
    wall limit exists only so a wedged or crashed run terminates.
    '''

    def __init__(self, clock, sim_limit, wall_limit, desc):
        self.clock = clock
        self.sim_limit = sim_limit
        self.wall_limit = wall_limit
        self.desc = desc
        self.sim0 = clock.now() if clock is not None else None
        self.wall0 = time.time()

    def sim_elapsed(self):
        if self.clock is None or self.sim0 is None:
            return 0.0
        now = self.clock.now()
        return 0.0 if now is None else now - self.sim0

    def wall_elapsed(self):
        return time.time() - self.wall0

    def check(self):
        '''raise TestFailure if this deadline has passed'''
        if self.sim0 is not None and self.sim_elapsed() > self.sim_limit:
            raise TestFailure("'%s' timed out after %.0f sim-seconds"
                              % (self.desc, self.sim_elapsed()))
        if self.wall_elapsed() > self.wall_limit:
            raise TestFailure(
                "'%s' hit the %.0fs wall-clock backstop after only %.0f "
                "sim-seconds - the host is running far below the requested "
                "speedup" % (self.desc, self.wall_limit, self.sim_elapsed()))


class SpeedupMeter(object):
    '''achieved speedup: simulated seconds advanced per wall second'''

    def __init__(self, want):
        self.want = want
        self.win_start = None
        self.latest = None
        self.overall_start = None
        self.steady_start = None

    def update(self, sim_s, wall_s):
        if self.win_start is None:
            self.win_start = (sim_s, wall_s)
            self.overall_start = (sim_s, wall_s)
        self.latest = (sim_s, wall_s)

    def mark_steady(self):
        # start of the converged region: the adaptive governor needs one
        # dwell to settle, so the back half of the follow phase measures
        # what the host actually sustains once tuned - the honest
        # "capable of Nx" figure for a long swarm run, free of the
        # boot/takeoff transient that dominates a short test's average
        if self.steady_start is None:
            self.steady_start = self.latest

    def steady(self):
        return self._rate(self.steady_start, self.latest)

    @staticmethod
    def _rate(a, b):
        if a is None or b is None:
            return None
        d_wall = b[1] - a[1]
        if d_wall < 1e-6:
            return None
        return (b[0] - a[0]) / d_wall

    def window(self):
        return self._rate(self.win_start, self.latest)

    def overall(self):
        return self._rate(self.overall_start, self.latest)

    def start_new_window(self):
        self.win_start = self.latest


class Vehicle(object):
    '''one SITL process plus its MAVLink connection'''

    # per-model default parameter file inside the binary's ROMFS; the
    # fast-swarm overrides are chained after it via --defaults
    MODEL_DEFAULTS = {
        "plane": "@ROMFS/models/plane.parm",
        "+": "@ROMFS/default_params/copter.parm",
    }

    def __init__(self, name, binary, instance, sysid, model, cluster, speedup,
                 logdir, opts):
        self.name = name
        self.binary = binary
        self.instance = instance
        self.sysid = sysid
        self.model = model
        self.cluster = cluster
        self.speedup = speedup
        self.logdir = logdir
        self.opts = opts
        self.proc = None
        self.workdir = None
        self.logfile = None
        self.mav = None
        self.clock = None       # set once the cluster segment exists
        # Drains EVERY vehicle's link, not just this one. A SITL whose
        # socket is not read will fill its TCP send buffer and block,
        # which freezes its simulated clock - so any wait longer than a
        # moment must keep every link moving, not only the one it
        # is talking to.
        self.drain_hook = None

    @property
    def port(self):
        return 5760 + 10 * self.instance

    def start(self):
        # Each SITL needs its OWN working directory: the parameter store is
        # a fixed "eeprom.bin" in the cwd (HAL_STORAGE_FILE), so sharing a
        # directory means several vehicles fighting over one file. This is why
        # sim_vehicle.py builds per-instance directories too.
        self.workdir = os.path.join(self.logdir, self.name)
        os.makedirs(self.workdir, exist_ok=True)
        self.logfile = open(os.path.join(self.logdir, "%s.log" % self.name), "wb")
        defaults = self.MODEL_DEFAULTS[self.model] + "," + self.opts.fast_parm
        if self.opts.loop_parm:
            defaults += "," + self.opts.loop_parm
        cfg = os.path.join(self.logdir, "%s_cfg.parm" % self.name)
        if os.path.exists(cfg):
            defaults += "," + cfg
        cmd = [
            self.binary,
            "--instance", str(self.instance),
            "--cluster=%d" % self.cluster,
            "--sysid", str(self.sysid),
            "--model", self.model,
            "--speedup", str(self.speedup),
            "--home", HOME,
            "--wipe",
            # chain the model's own defaults with the fast-swarm
            # overrides (EKF disabled, logging off, terrain off) written
            # by the test - measured at ~44 percent more achieved
            # speedup, and on Windows it also removes the log_io
            # thread's ~15k sleeps/second, each of which costs a full
            # 1ms timer tick under Cygwin
            "--defaults", defaults,
        ]
        # note: lowering the physics frame rate (--rate 400) was tried
        # here and MEASURED SLOWER (23.9x -> 13.6x): 1:1 physics frames
        # per vehicle-loop iteration interact badly with the sample
        # pacing. Deliberately not used.
        env = dict(os.environ)
        # disable the per-task stack NaN poisoning, a SITL debug aid
        # measured at ~4 percent of CPU (see AP_Scheduler.cpp)
        env["SITL_DISABLE_STACK_NANF"] = "1"
        # remove every wall-clock sleep from the sim path; cores run hot
        env["SITL_HARD_NONBLOCK"] = "1"
        # runtime governor: trade physics frame rate for achieved speedup
        # until the commanded speedup is met (measured: the curve is
        # non-monotonic, so it must be tuned live, not statically)
        env["SITL_ADAPTIVE_RATE"] = "1"
        self.proc = subprocess.Popen(
            cmd, stdout=self.logfile, stderr=subprocess.STDOUT,
            cwd=self.workdir, preexec_fn=os.setsid, env=env)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def connect(self, timeout=60):
        # wall-clock: a TCP connect and the first heartbeat are not
        # simulation-paced, and the cluster clock is not readable yet
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                self.mav = mavutil.mavlink_connection(
                    "tcp:127.0.0.1:%u" % self.port, source_system=250,
                    autoreconnect=True)
                break
            except OSError:
                # port not accepting connections yet
                time.sleep(0.5)
        if self.mav is None:
            raise TestFailure("%s: could not connect on port %u"
                              % (self.name, self.port))
        if self.mav.wait_heartbeat(timeout=timeout) is None:
            raise TestFailure("%s: no heartbeat" % self.name)
        for _mid in (mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
                     mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                     mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                _mid, 1000000, 0, 0, 0, 0, 0)
        # every STATUSTEXT and command ACK, whoever parses the packet:
        # when a vehicle refuses to fly it says why here and nowhere else
        self.mav.message_hooks.append(self._msg_hook)
        # Ask for only what this test reads, at a modest rate. These rates
        # are in SIM Hz, so the wall-clock message rate is rate * speedup;
        # requesting everything at 20Hz would be 2000 messages/second per
        # vehicle if the host ever did achieve 100x.
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
        for stream, rate in (
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2)):
            self.mav.mav.request_data_stream_send(
                self.mav.target_system, self.mav.target_component,
                stream, rate, 1)

    def deadline(self, sim_limit, desc):
        return Deadline(self.clock, sim_limit, self.opts.wall_limit,
                        "%s: %s" % (self.name, desc))

    def set_param(self, name, value, sim_limit=60):
        dl = self.deadline(sim_limit, "set %s" % name)
        seen = None
        while True:
            self.mav.mav.param_set_send(
                self.mav.target_system, self.mav.target_component,
                name.encode("utf-8"),
                float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True,
                                      timeout=2)
            if msg is not None and msg.param_id == name:
                if abs(msg.param_value - float(value)) < 1e-4:
                    return
                # the vehicle answered with a different value: almost always
                # the parameter clamping an out-of-range request
                seen = msg.param_value
            if self.drain_hook is not None:
                self.drain_hook()
            try:
                dl.check()
            except TestFailure as e:
                if seen is not None:
                    raise TestFailure(
                        "%s: %s would not take %s - vehicle reports %g "
                        "(value out of range for this parameter?)"
                        % (self.name, name, value, seen))
                raise TestFailure(
                    "%s (parameter %s may not exist on this vehicle)"
                    % (e, name))

    def set_params(self, params):
        for name, value in params.items():
            self.set_param(name, value)

    def set_mode(self, mode, sim_limit=120):
        mapping = self.mav.mode_mapping()
        if mapping is None:
            # only populated once a heartbeat has told us the vehicle type
            raise TestFailure("%s: no mode mapping yet" % self.name)
        if mode not in mapping:
            raise TestFailure("%s: unknown mode %s" % (self.name, mode))
        want = mapping[mode]
        dl = self.deadline(sim_limit, "enter %s" % mode)
        while True:
            self.mav.mav.set_mode_send(
                self.mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                want)
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg is not None and msg.custom_mode == want:
                return
            if self.drain_hook is not None:
                self.drain_hook()
            dl.check()

    def wait_ready_to_arm(self, sim_limit=None):
        # phase deadlines bound WALL time uniformly under the pinned
        # wall-slaved schedule: a fixed sim window evaporates in
        # wall-instants once the schedule engages
        if sim_limit is None:
            sim_limit = 30 * max(1, self.opts.speedup)
        # this is really waiting for the EKF and GPS to converge, which is
        # paced by simulated time, so the limit must be in sim seconds
        dl = self.deadline(sim_limit, "become armable")
        while True:
            msg = self.mav.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
            if msg is not None:
                bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                if msg.onboard_control_sensors_health & bit:
                    return
            if self.drain_hook is not None:
                self.drain_hook()
            dl.check()

    def arm(self, sim_limit=None):
        if sim_limit is None:
            sim_limit = 15 * max(1, self.opts.speedup)
        dl = self.deadline(sim_limit, "arm")
        while True:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if (msg is not None and
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
            if self.drain_hook is not None:
                self.drain_hook()
            dl.check()

    def _msg_hook(self, _conn, msg):
        t = msg.get_type()
        if t == "STATUSTEXT":
            progress("%s says: %s" % (self.name, msg.text))
        elif t == "COMMAND_ACK":
            progress("%s ack: cmd=%u result=%u"
                     % (self.name, msg.command, msg.result))

    def takeoff_cmd(self, alt):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt)

    def position(self):
        '''latest GLOBAL_POSITION_INT as (lat, lng, alt_m, rel_alt_m, hdg)'''
        msg = self.mav.messages.get("GLOBAL_POSITION_INT", None)
        if msg is None:
            return None
        return (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3,
                msg.relative_alt * 1e-3, msg.hdg * 1e-2)

    def rel_alt(self):
        pos = self.position()
        return pos[3] if pos is not None else 0.0

    def drain(self):
        while True:
            if self.mav.recv_match(blocking=False) is None:
                return

    def stop(self):
        if self.mav is not None:
            try:
                self.mav.close()
            except OSError:
                pass
        if self.proc is not None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                # already gone, or not ours to signal
                pass
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
        if self.logfile is not None:
            self.logfile.close()


def offset_location(lat, lng, north_m, east_m):
    '''shift a lat/lng by a north/east offset in metres'''
    dlat = north_m / 1.113195e5
    dlng = east_m / (1.113195e5 * math.cos(math.radians(lat)))
    return (lat + dlat, lng + dlng)


def formation_target(ppos, offset):
    '''where a copter is commanded to sit, for FOLL_OFS_TYPE=1

    The offset is expressed relative to the lead vehicle's heading (x
    forward, y right), so it has to be rotated into north/east before it
    means anything on the ground.
    '''
    ofs_x, ofs_y, _ = offset
    hdg = math.radians(ppos[4])
    north = ofs_x * math.cos(hdg) - ofs_y * math.sin(hdg)
    east = ofs_x * math.sin(hdg) + ofs_y * math.cos(hdg)
    return offset_location(ppos[0], ppos[1], north, east)


def horizontal_distance(a, b):
    '''metres between two (lat, lng, ...) tuples'''
    dlat = a[0] - b[0]
    dlng = (a[1] - b[1]) * math.cos(math.radians((a[0] + b[0]) * 0.5))
    return math.sqrt(dlat ** 2 + dlng ** 2) * 1.113195e5


class ClusterFollowTest(object):

    # formation offsets in metres relative to the plane's heading: one
    # astern, one off each wing, all below the plane (FOLL_OFS_Z is +ve down)
    # Line astern, not line abreast. The plane holds a circular loiter, so
    # a port slot and a starboard slot are not equivalent: one sits inside
    # the turn and one outside, and they track very differently (measured
    # 240m vs 100m steady-state error). Stacking both copters directly
    # behind puts them on the same ground track as the plane, so the test
    # measures follow accuracy rather than which side of the circle a
    # copter was assigned to.
    OFFSETS = {
        "copter1": (-60.0, 0.0, 20.0),
        "copter2": (-120.0, 0.0, 20.0),
    }

    def __init__(self, opts):
        self.opts = opts
        self.logdir = tempfile.mkdtemp(prefix="/tmp/sitl_cluster_follow_")
        # a previous run killed mid-flight (SIGKILL skips the vehicles'
        # own cleanup) leaves this cluster's shared-memory segment
        # holding a dead schedule epoch and armed flags; vehicles
        # starting now would join it. Retire it up front, and again in
        # cleanup() so this run leaves nothing behind either.
        self.purge_segment()
        # overrides chained after each vehicle's model defaults: no EKF
        # (SITL perfect AHRS), no dataflash logging, no terrain grid
        opts.fast_parm = os.path.abspath(
            os.path.join(self.logdir, "fast_swarm.parm"))
        with open(opts.fast_parm, "w") as f:
            # EKF3 runs in the background even with AHRS_EKF_TYPE 10
            # selecting the SITL estimator, and default telemetry
            # stream rates generate MAVLink per SIM second - both are
            # dead weight the vehicles at the governor's frame-rate
            # floor cannot afford. The bridge only needs position and
            # status streams.
            f.write("AHRS_EKF_TYPE 10\n"
                    "LOG_BACKEND_TYPE 0\n"
                    "TERRAIN_ENABLE 0\n"
                    "EK3_ENABLE 0\n"
                    "EK2_ENABLE 0\n"
                    "SR0_RAW_SENS 0\n"
                    "SR0_EXT_STAT 1\n"
                    "SR0_RC_CHAN 0\n"
                    "SR0_RAW_CTRL 0\n"
                    "SR0_POSITION 1\n"
                    "SR0_EXTRA1 0\n"
                    "SR0_EXTRA2 0\n"
                    "SR0_EXTRA3 0\n"
                    "SR0_ADSB 0\n"
                    "SIM_FLOAT_EXCEPT 0\n"
                    "ARMING_CHECK 0\n"
                    # the simulated board heats over minutes of SIM time and
                    # drifts the baro; racing sim time puts the whole warmup
                    # between baro cal and arming, so a grounded copter reads
                    # metres of altitude, believes it is airborne, and
                    # refuses its guided takeoff. No thermal drift.
                    "SIM_TEMP_BFACTOR 0\n"
                    "SIM_TEMP_TCONST 1\n"
                    # the battery model's internal resistance grows with
                    # temperature; racing sim time cooks it in wall-seconds
                    # and sag collapses thrust to 1g at 100% throttle -
                    # copters hung mid-air unable to climb. No sag, ever.
                    "SIM_BATT_RES_OHM 0\n")
        # Only when chasing real speedup (> 5): run every vehicle's
        # flight-controller loop at 200Hz. Scheduler iterations are the
        # measured scaling axis for per-sim-second cost, and the loop
        # rate also sets the adaptive governor's physics-rate floor
        # (3x the loop rate): at ArduPlane's default 300Hz loop the
        # plane floors at 900 physics frames while the 200Hz copters
        # sit at 600, making the plane the most expensive cluster
        # member for nothing the swarm test measures. 200Hz for a plane
        # is repo-precedented (glider.parm). Sub-5x runs are untouched.
        opts.loop_parm = ""
        if opts.speedup > 5:
            opts.loop_parm = os.path.abspath(
                os.path.join(self.logdir, "swarm_200hz.parm"))
            with open(opts.loop_parm, "w") as f:
                f.write(
                    # loop budget matches the actual stretched step
                    # (~10ms): at 200Hz the scheduler saw every
                    # 5ms-budget loop consume 11ms of sim time and
                    # permanently shed its table tasks - the plane's
                    # servo output froze (20km open-loop climb) and
                    # the copters' takeoff machinery ran starved
                    "SCHED_LOOP_RATE 100\n")
        # every mission parameter is baked into per-vehicle defaults
        # files loaded at boot: the old MAVLink configuration phase was
        # ~40 parameter round-trips per vehicle of pure wall time
        # before a single simulated second of the mission ran
        with open(os.path.join(self.logdir, "plane_cfg.parm"), "w") as f:
            f.write("AIRSPEED_CRUISE 12\n"
                    "AIRSPEED_MIN 9\n"
                    "AIRSPEED_MAX 16\n"
                    "WP_LOITER_RAD 250\n"
                    "TKOFF_ALT 100\n")
        for cname, ofs in self.OFFSETS.items():
            with open(os.path.join(self.logdir,
                                   "%s_cfg.parm" % cname), "w") as f:
                f.write("DISARM_DELAY 0\n"
                        "FS_CRASH_CHECK 0\n"
                        "SRTL_POINTS 0\n"
                        "FOLL_ENABLE 1\n"
                        "FOLL_SYSID 1\n"
                        "FOLL_OFS_TYPE 1\n"
                        "FOLL_OFS_X %.1f\n"
                        "FOLL_OFS_Y %.1f\n"
                        "FOLL_OFS_Z %.1f\n"
                        "FOLL_ALT_TYPE 0\n"
                        "FOLL_DIST_MAX 1000\n"
                        "FOLL_TIMEOUT %.0f\n"
                        "FOLL_YAW_BEHAVE 1\n"
                        "WP_SPD 20\n"
                        "WP_SPD_UP 5\n"
                        "WP_SPD_DN 5\n"
                        "WP_ACC 5\n"
                        % (ofs[0], ofs[1], ofs[2], FOLL_TIMEOUT_S))
        self.plane = Vehicle("plane", opts.plane_binary, 0, 1, "plane",
                             opts.cluster, opts.speedup, self.logdir, opts)
        self.copters = [
            Vehicle("copter%u" % i, opts.copter_binary, i, i + 1, "+",
                    opts.cluster, opts.speedup, self.logdir, opts)
            for i in (1, 2)
        ]
        self.vehicles = [self.plane] + self.copters
        self.instances = [v.instance for v in self.vehicles]
        self.view = None
        self.meter = SpeedupMeter(opts.speedup)
        self.worst_skew_us = 0
        self.max_bridge_gap_sim_s = 0.0
        self.last_bridge_sim_s = None
        self.phase = "init"
        self.last_report_wall = 0.0
        self.last_temp_wall = 0.0

    def drain_all(self):
        '''read every link, so no vehicle blocks on a full send buffer'''
        for v in self.vehicles:
            if v.mav is not None:
                v.drain()

    def check_cluster(self):
        '''assert the cluster is intact and within skew; returns sim seconds'''
        members = self.view.members()
        if len(members) != len(self.vehicles):
            missing = sorted(set(self.instances) - set(members))
            raise TestFailure("cluster lost instance(s) %s mid-test" % missing)
        times = list(members.values())
        skew = max(times) - min(times)
        self.worst_skew_us = max(self.worst_skew_us, skew)
        if skew > self.opts.max_skew_us:
            raise TestFailure("cluster skew %u us exceeded limit %u us"
                              % (skew, self.opts.max_skew_us))
        return min(times) * 1e-6

    def pump(self, until, sim_limit, desc, bridge=True):
        '''run the bridge and telemetry until until() returns True

        sim_limit is in simulated seconds. Returns the simulated seconds
        that elapsed.
        '''
        self.phase = desc
        dl = Deadline(self.view, sim_limit, self.opts.wall_limit, desc)

        while True:
            for v in self.vehicles:
                if not v.alive():
                    raise TestFailure("%s exited during '%s'" % (v.name, desc))
                v.drain()

            sim_s = self.check_cluster()
            if bridge:
                self.bridge(sim_s)
            self.meter.update(sim_s, time.time())
            self.report(sim_s)

            if until(sim_s):
                return dl.sim_elapsed()

            dl.check()
            time.sleep(self.opts.loop_dt)

    def bridge(self, sim_s):
        '''forward the plane's position onto each copter's link'''
        pmsg = self.plane.mav.messages.get("GLOBAL_POSITION_INT", None)
        if pmsg is None:
            return
        for c in self.copters:
            # preserve the plane's sysid in the header - that is what
            # AP_Follow matches against FOLL_SYSID
            c.mav.mav.srcSystem = self.plane.sysid
            c.mav.mav.global_position_int_send(
                pmsg.time_boot_ms, pmsg.lat, pmsg.lon, pmsg.alt,
                pmsg.relative_alt, pmsg.vx, pmsg.vy, pmsg.vz, pmsg.hdg)
            c.mav.mav.srcSystem = 250

        # how stale the copters' view of the plane gets, in SIM time - this
        # is the quantity FOLL_TIMEOUT is measured against, and the one that
        # degrades as speedup rises
        if self.last_bridge_sim_s is not None:
            gap = sim_s - self.last_bridge_sim_s
            self.max_bridge_gap_sim_s = max(self.max_bridge_gap_sim_s, gap)
        self.last_bridge_sim_s = sim_s

    def report(self, sim_s):
        # wall-clock interval on purpose: this paces console output for a
        # human, and must not speed up or slow down with the simulation
        wall_s = time.time()
        if wall_s - self.last_report_wall < self.opts.report_interval:
            return
        self.last_report_wall = wall_s
        if wall_s - self.last_temp_wall >= 2.0:
            self.last_temp_wall = wall_s
            temp = cpu_temp_report()
            if temp is not None:
                progress(temp)
        now = self.meter.window()
        overall = self.meter.overall()
        self.meter.start_new_window()
        pct = (now / self.opts.speedup * 100.0) if now else 0.0
        dists = ""
        ppos = self.plane.position()
        if ppos is not None:
            parts = []
            for c in self.copters:
                cpos = c.position()
                if cpos is None:
                    parts.append("%s=?" % c.name)
                    continue
                want = formation_target(ppos, self.OFFSETS[c.name])
                gpi = c.mav.messages.get("GLOBAL_POSITION_INT", None)
                hb = c.mav.messages.get("HEARTBEAT", None)
                vz = (-gpi.vz * 1e-2) if gpi is not None else float("nan")
                armed = (bool(hb.base_mode
                              & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                         if hb is not None else None)
                mode = hb.custom_mode if hb is not None else None
                parts.append("%s err=%.0fm alt=%.0fm vz=%+.1f mode=%s%s" % (
                    c.name, horizontal_distance(want, cpos), cpos[3], vz,
                    mode, "" if armed else " DISARMED"))
            phb = self.plane.mav.messages.get("HEARTBEAT", None)
            pnav = self.plane.mav.messages.get(
                "NAV_CONTROLLER_OUTPUT", None)
            phud = self.plane.mav.messages.get("VFR_HUD", None)
            patt = self.plane.mav.messages.get("ATTITUDE", None)
            dists = " plane_alt=%.0fm mode=%s alt_err=%s thr=%s pit=%s " % (
                ppos[3],
                phb.custom_mode if phb is not None else "?",
                "%.0f" % pnav.alt_error if pnav is not None else "?",
                "%s/as=%.0f" % (phud.throttle, phud.airspeed)
                if phud is not None else "?",
                "%.0f" % math.degrees(patt.pitch)
                if patt is not None else "?")
            dists += " ".join(parts)
        progress("[%s] sim=%.0fs speedup want=%ux now=%s avg=%s (%.0f%% of "
                 "target) skew=%uus%s"
                 % (self.phase, sim_s, self.opts.speedup,
                    "%.1fx" % now if now else "?",
                    "%.1fx" % overall if overall else "?",
                    pct, self.worst_skew_us, dists))

    def run(self):
        for v in self.vehicles:
            progress("starting %s (instance %u, sysid %u) at %ux speedup"
                     % (v.name, v.instance, v.sysid, v.speedup))
            v.start()
        # wall-clock: process startup is not paced by the simulation
        time.sleep(5)
        for v in self.vehicles:
            v.connect()
            progress("%s connected on port %u" % (v.name, v.port))

        self.view = ClusterView(self.opts.cluster, self.instances)
        for v in self.vehicles:
            v.clock = self.view
            v.drain_hook = self.drain_all
        members = self.view.members()
        if len(members) != len(self.vehicles):
            raise TestFailure("only %u of %u vehicles joined cluster %u"
                              % (len(members), len(self.vehicles),
                                 self.opts.cluster))
        progress("all %u vehicles registered in cluster %u"
                 % (len(members), self.opts.cluster))

        # mission parameters were baked into per-vehicle defaults files
        # at spawn - no MAVLink configuration phase

        for v in self.vehicles:
            progress("%s waiting for armable state" % v.name)
            v.wait_ready_to_arm()

        # get the plane airborne and circling first, so the copters have a
        # real moving target to acquire
        progress("launching plane")
        self.plane.set_mode("TAKEOFF")
        self.plane.arm()
        self.pump(lambda s: self.plane.rel_alt() > 80,
                  sim_limit=30 * max(1, self.opts.speedup),
                  desc="plane-takeoff", bridge=False)
        progress("plane airborne at %.0fm, switching to LOITER"
                 % self.plane.rel_alt())
        self.plane.set_mode("LOITER")

        progress("launching copters")
        for c in self.copters:
            c.set_mode("GUIDED")
            c.arm()
            c.takeoff_cmd(60)
        self.pump(lambda s: all(c.rel_alt() > 50 for c in self.copters),
                  sim_limit=30 * max(1, self.opts.speedup),
                  desc="copter-takeoff")
        progress("copters airborne at %s"
                 % ["%.0fm" % c.rel_alt() for c in self.copters])

        for c in self.copters:
            c.set_mode("FOLLOW")
        progress("all copters in FOLLOW, tracking for %.0f sim-seconds"
                 % self.opts.follow_time)

        # Formation error - how far each copter is from the position it is
        # actually commanded to hold - is the honest measure. Distance to
        # the plane is not: the plane orbits at WP_LOITER_RAD, so a copter
        # that never moved would still be "near" it for much of each lap.
        closest = {c.name: None for c in self.copters}
        # Longest CONTINUOUS period inside the formation tolerance, not the
        # total. A copter that never took off would still accumulate plenty
        # of total time "in tolerance" as the plane's circular loiter swept
        # past it once a lap; it could not stay in tolerance continuously.
        holding = {c.name: 0.0 for c in self.copters}
        settled = {c.name: 0.0 for c in self.copters}
        last = {"sim": None}

        follow_start = {"sim": None}

        def track(sim_s):
            # accumulate station-keeping in SIM seconds, so "held for 60s"
            # means the same amount of flying at any achieved speedup
            dt = 0.0 if last["sim"] is None else sim_s - last["sim"]
            last["sim"] = sim_s
            if follow_start["sim"] is None:
                follow_start["sim"] = sim_s
            elif sim_s - follow_start["sim"] > self.opts.follow_time / 2.0:
                self.meter.mark_steady()
            ppos = self.plane.position()
            if ppos is None:
                return False
            for c in self.copters:
                cpos = c.position()
                if cpos is None:
                    continue
                want = formation_target(ppos, self.OFFSETS[c.name])
                err = horizontal_distance(want, cpos)
                if closest[c.name] is None or err < closest[c.name]:
                    closest[c.name] = err
                if err < self.opts.hold_distance:
                    holding[c.name] += dt
                    settled[c.name] = max(settled[c.name], holding[c.name])
                else:
                    holding[c.name] = 0.0
            return False        # never satisfied: run for the full duration

        try:
            self.pump(track, sim_limit=self.opts.follow_time, desc="follow")
        except TestFailure as e:
            if "timed out after" not in str(e):
                raise           # a real failure, not the intended duration

        achieved = self.meter.overall()
        progress("achieved speedup over the run: %s (requested %ux)"
                 % ("%.1fx" % achieved if achieved else "?", self.opts.speedup))
        steady = self.meter.steady()
        if steady:
            progress("steady-state speedup (back half of follow): %.1fx"
                     % steady)
        # surface each vehicle's adaptive-rate governor decisions so a CI
        # log shows what frame rate every vehicle settled at
        for v in self.vehicles:
            try:
                with open(os.path.join(self.logdir, "%s.log" % v.name),
                          "rb") as f:
                    lines = [ln.decode("utf-8", "replace").strip()
                             for ln in f if b"adaptive rate" in ln]
            except OSError:
                lines = []
            if lines:
                for ln in lines[-4:]:
                    progress("%s governor: %s" % (v.name, ln))
            else:
                progress("%s governor: no rate changes" % v.name)
        # any shortfall from the commanded speedup is a failure: the
        # adaptive governor is required to slow the physics until the
        # commanded speedup is reached. Checked after the governor
        # traces print so a failure is diagnosable.
        if steady and steady < self.opts.speedup:
            raise TestFailure(
                "achieved steady-state speedup %.1fx is below the "
                "commanded %ux" % (steady, self.opts.speedup))
        progress("worst cluster skew: %u us" % self.worst_skew_us)
        progress("worst gap between position updates: %.2f sim-seconds "
                 "(FOLL_TIMEOUT is %.0fs)"
                 % (self.max_bridge_gap_sim_s, FOLL_TIMEOUT_S))
        if self.max_bridge_gap_sim_s > FOLL_TIMEOUT_S:
            raise TestFailure(
                "the telemetry bridge fell behind: %.1f sim-second gap "
                "exceeds FOLL_TIMEOUT %.0fs, so the copters lost the plane. "
                "Reduce --speedup or --loop-dt."
                % (self.max_bridge_gap_sim_s, FOLL_TIMEOUT_S))

        for c in self.copters:
            progress("%s: best formation error %s, held station %.0f sim-seconds unbroken"
                     % (c.name,
                        "%.0fm" % closest[c.name] if closest[c.name] else "never",
                        settled[c.name]))
        for c in self.copters:
            if closest[c.name] is None or closest[c.name] > self.opts.hold_distance:
                raise TestFailure(
                    "%s never reached its formation slot within %.0fm "
                    "(best error %s)"
                    % (c.name, self.opts.hold_distance,
                       "%.0fm" % closest[c.name] if closest[c.name] else "never"))
            if settled[c.name] < self.opts.min_hold_time:
                raise TestFailure(
                    "%s only held formation for %.0f sim-seconds unbroken, "
                    "wanted %.0f"
                    % (c.name, settled[c.name], self.opts.min_hold_time))

        progress("PASS: %u copters followed the plane in cluster %u"
                 % (len(self.copters), self.opts.cluster))

    def purge_segment(self):
        if sys.platform in ("cygwin", "win32"):
            # Windows shared memory is a kernel object that vanishes
            # with its last handle - nothing can go stale
            return
        seg = "/dev/shm/ardupilot_sitl_cluster_%u_%u" % (
            os.getuid(), self.opts.cluster)
        try:
            os.unlink(seg)
            progress("removed stale cluster segment %s" % seg)
        except FileNotFoundError:
            pass
        except OSError as e:
            progress("could not remove %s: %s" % (seg, e))

    def cleanup(self):
        if self.view is not None:
            self.view.close()
        for v in self.vehicles:
            v.stop()
        self.purge_segment()


def main():
    parser = optparse.OptionParser("test_cluster_follow.py [options]")

    # ------------------------------------------------------------------
    # Settings: edit HERE, not via flags. Command-line options exist only
    # for the values genuinely varied per invocation (see the standing
    # no-speculative-arguments rule); everything else is a constant.
    # ------------------------------------------------------------------
    class Settings:
        # SITL binaries to launch
        copter_binary = "build/sitl/bin/arducopter"
        plane_binary = "build/sitl/bin/arduplane"
        # shared-memory cluster id; two simultaneous runs with the same
        # id join EACH OTHER's cluster and produce chaos
        cluster = 91
        # bridge/telemetry pump period, WALL seconds. speedup * loop_dt
        # is the sim-time gap between the position updates the copters
        # see; keep it well under FOLL_TIMEOUT (30s)
        loop_dt = 0.01
        # WALL seconds between progress lines
        report_interval = 2.0
        # metres of formation error that still counts as holding station
        hold_distance = 250.0
        # max tolerated cluster clock skew in sim microseconds;
        # None = 10000 * speedup (the barrier window is wall-fixed)
        max_skew_us = None
        # SIM seconds to hold formation for, eg 600=10 virtual minutes, so a speedup 100x results a
        # in 6s real human elapsed time.
        follow_time = 600.0
        # SIM seconds each copter must hold formation for WITHOUT a
        # break; the plane laps its loiter in ~130s, so this cannot be
        # satisfied by a copter merely being orbited past
        min_hold_time = 120.0
        # wall-clock backstop per phase, seconds; only stops a wedged
        # or starved run
        wall_limit = 80.0

    parser.add_option("", "--speedup", type="int", default=50,
                      help="requested SITL speedup for every vehicle in "
                           "the cluster; the achieved rate is measured, "
                           "reported and gated")
    opts, _ = parser.parse_args()
    for _k in ("copter_binary", "plane_binary", "cluster", "loop_dt",
               "report_interval", "hold_distance", "max_skew_us",
               "follow_time", "min_hold_time", "wall_limit"):
        setattr(opts, _k, getattr(Settings, _k))

    if opts.max_skew_us is None:
        # twice the barrier's own window. The barrier decimates crossings
        # to every 4*speedup frames and scales its wall-fixed window
        # accordingly (SIM_Aircraft.cpp: 5000us * 4 * speedup), so the
        # assertion doubles that to allow one full window of drift
        # between crossings
        opts.max_skew_us = 2 * 5000 * 4 * opts.speedup

    # resolve now: each SITL is spawned with cwd set to its log directory,
    # so a relative binary path would be looked up in the wrong place
    opts.copter_binary = os.path.abspath(opts.copter_binary)
    opts.plane_binary = os.path.abspath(opts.plane_binary)
    for path in (opts.copter_binary, opts.plane_binary):
        if not os.path.exists(path):
            print("FAIL: missing binary %s "
                  "(./waf configure --board sitl && ./waf copter plane)" % path)
            return 1

    gap = opts.speedup * opts.loop_dt
    if gap > FOLL_TIMEOUT_S / 2:
        print("FAIL: --speedup %u with --loop-dt %.3f gives a %.1f sim-second "
              "gap between position updates; FOLL_TIMEOUT is %.0fs so the "
              "copters would lose the plane. Reduce --loop-dt to %.3f or less."
              % (opts.speedup, opts.loop_dt, gap, FOLL_TIMEOUT_S,
                 FOLL_TIMEOUT_S / 2 / opts.speedup))
        return 1

    governors = set_cpu_governor_performance()

    test = ClusterFollowTest(opts)
    try:
        test.run()
        return 0
    except TestFailure as e:
        progress("FAIL: %s" % e)
        progress("logs kept in %s" % test.logdir)
        return 1
    except KeyboardInterrupt:
        progress("interrupted")
        return 1
    finally:
        test.cleanup()
        restore_cpu_governors(governors)


if __name__ == "__main__":
    sys.exit(main())
