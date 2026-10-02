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

This script never reads the shared-memory segment itself - the clustering
is entirely AP_SITL_SharedMem's business. Vehicle output goes to
per-vehicle log files in the run directory (Windows console writes are
synchronous and slow enough to brake the sim, and QuickEdit selection
freezes every process sharing the console); the end-of-run summary
prints each vehicle's governor decisions and achieved speedup from
those logs. Python's job is only to launch, configure, fly and observe
over MAVLink; all its timeouts and measurements are plain wall-clock
seconds.

The test asserts one thing: follow works - each copter closes on its
formation slot behind the plane and holds it.

AP_FLAKE8_CLEAN
'''

import math
import os
import subprocess
import sys
import tempfile
import time

from pymavlink import mavutil

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


HIGH_PERFORMANCE_GUID = "8c5e7fda-e8bf-4a96-9a85-a6e23a8c635c"


def set_windows_power_performance():
    """best effort: switch to the High performance power plan for the run

    The Windows equivalent of Linux's CPU frequency governor: on the
    default Balanced plan the clocks sag exactly when several busy sim
    processes need them. Returns the previous plan's GUID for
    restore_windows_power(), or None if nothing was changed.
    """
    if sys.platform != "win32":
        return None
    import re
    try:
        out = subprocess.check_output(["powercfg", "/getactivescheme"])
        m = re.search(r"([0-9a-fA-F-]{36})", out.decode("utf-8", "replace")
                      if isinstance(out, bytes) else out)
        previous = m.group(1) if m else None
        if previous and previous.lower() == HIGH_PERFORMANCE_GUID:
            return None                 # already there, nothing to restore
        subprocess.check_call(["powercfg", "/setactive",
                               HIGH_PERFORMANCE_GUID])
        progress("Windows power plan -> High performance")
        return previous
    except (OSError, subprocess.CalledProcessError):
        progress("NOTE: could not switch to the High performance power "
                 "plan; expect lower and noisier speedups - run "
                 "'powercfg /setactive %s' by hand" % HIGH_PERFORMANCE_GUID)
        return None


def disable_power_throttling(binaries):
    """best effort: opt each SITL exe out of Windows EcoQoS throttling

    Windows 10 1709+ classifies windowless console processes as
    background work and applies EcoQoS "efficiency mode": low clocks,
    efficiency cores on hybrid CPUs, and a coalesced 15.6ms timer tick
    even when the process asked for 1ms. Measured on an 8-core box: each
    vehicle pinned at roughly a QUARTER of one core by this alone.
    'powercfg /powerthrottling disable /path <exe>' turns it off
    per-executable and persists, so it also fixes runs started by hand.
    Verify in Task Manager Details: the "Power throttling" column for
    each vehicle should read Disabled (no green leaf).
    """
    if sys.platform != "win32":
        return
    for b in binaries:
        path = os.path.abspath(b)
        try:
            subprocess.check_call(["powercfg", "/powerthrottling",
                                   "disable", "/path", path])
            progress("EcoQoS power throttling disabled for %s"
                     % os.path.basename(path))
        except (OSError, subprocess.CalledProcessError):
            progress("NOTE: could not disable power throttling for %s - "
                     "run by hand (may need admin): powercfg "
                     "/powerthrottling disable /path \"%s\"" % (
                         os.path.basename(path), path))


def restore_windows_power(previous):
    if previous is None:
        return
    try:
        subprocess.check_call(["powercfg", "/setactive", previous])
        progress("Windows power plan restored")
    except (OSError, subprocess.CalledProcessError):
        pass


def progress(text):
    print("CLUSTER-FOLLOW: %s" % text)
    sys.stdout.flush()


class Deadline(object):
    """a plain wall-clock timeout for one phase"""

    def __init__(self, limit_s, desc):
        self.limit_s = limit_s
        self.desc = desc
        self.wall0 = time.time()

    def elapsed(self):
        return time.time() - self.wall0

    def check(self):
        if self.elapsed() > self.limit_s:
            raise TestFailure("'%s' timed out after %.0f seconds"
                              % (self.desc, self.elapsed()))


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
        self.mav = None
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
        os.makedirs(self.workdir)
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
        # SITL_HARD_NONBLOCK is deliberately NOT set: measured on the
        # Windows CI runner, the no-spin C++ smoke cluster sustains
        # 110x of the commanded speedup while the spinning follow
        # harness starves its own UART/IO threads so badly the config
        # phase cannot complete a parameter round-trip - three
        # vehicles of spin-yield threads on a small host choke the
        # threads they exist to serve. The in-binary 1ms timer
        # resolution fix removed the sleep-quantum penalty that spin
        # once compensated for.
        # runtime governor: trade physics frame rate for achieved speedup
        # until the commanded speedup is met (measured: the curve is
        # non-monotonic, so it must be tuned live, not statically)
        env["SITL_ADAPTIVE_RATE"] = "1"
        # vehicle output goes to a per-vehicle log file, NOT this
        # console: Windows console writes are synchronous and slow, and
        # three processes printing here measurably brakes the sim (and
        # a QuickEdit text selection freezes all of them). The governor
        # and speedup lines are summarised from the logs at the end.
        self.logfile = open(os.path.join(self.logdir, "%s.log" % self.name), "wb")
        self.proc = subprocess.Popen(cmd, cwd=self.workdir, env=env,
                                     stdout=self.logfile,
                                     stderr=subprocess.STDOUT)

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
            except (OSError, IOError):
                # Port not accepting connections yet. Both classes matter:
                # on Python 2.7 a refused TCP connect raises socket.error,
                # which is IOError, NOT OSError - they only became the same
                # class in Python 3.3.
                time.sleep(0.5)
        if self.mav is None:
            raise TestFailure("%s: could not connect on port %u"
                              % (self.name, self.port))
        if self.mav.wait_heartbeat() is None:
            raise TestFailure("%s: no heartbeat" % self.name)
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

    def deadline(self, timeout, desc):
        return Deadline(timeout, "%s: %s" % (self.name, desc))

    def set_param(self, name, value, timeout=60):
        dl = self.deadline(timeout, "set %s" % name)
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

    def set_mode(self, mode, timeout=120):
        mapping = self.mav.mode_mapping()
        if mapping is None:
            # only populated once a heartbeat has told us the vehicle type
            raise TestFailure("%s: no mode mapping yet" % self.name)
        if mode not in mapping:
            raise TestFailure("%s: unknown mode %s" % (self.name, mode))
        want = mapping[mode]
        dl = self.deadline(timeout, "enter %s" % mode)
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

    def wait_ready_to_arm(self, timeout=300):
        # this is really waiting for the EKF and GPS to converge, which is
        # paced by simulated time, so the limit must be in sim seconds
        dl = self.deadline(timeout, "become armable")
        while True:
            msg = self.mav.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
            if msg is not None:
                bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                if msg.onboard_control_sensors_health & bit:
                    return
            if self.drain_hook is not None:
                self.drain_hook()
            dl.check()

    def arm(self, timeout=120):
        dl = self.deadline(timeout, "arm")
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
            except (OSError, IOError):    # socket.error is IOError on py2.7
                pass
        if self.proc is not None:
            # Portable teardown. os.killpg + SIGKILL are POSIX-only and
            # proc.wait(timeout=) is Python 3 only, so neither can be used
            # here. Popen.kill() exists in Python 2.6+ and maps to
            # TerminateProcess on Windows, and the plain wait() reaps the
            # process afterwards. Leaving these out is what strands SITL
            # processes and causes 'Address already in use' on the next run.
            try:
                if self.proc.poll() is None:
                    self.proc.kill()
            except OSError:
                pass                # already gone
            try:
                self.proc.wait()
            except OSError:
                pass


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
        # Per-vehicle working directories only. SITL keeps its parameter
        # store in a fixed "eeprom.bin" in the cwd, so the vehicles cannot
        # share one. Vehicle output goes to per-vehicle log files here.
        self.logdir = tempfile.mkdtemp(prefix="sitl_cluster_follow_", dir=".")
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
        self.phase = "init"
        self.t0 = time.time()
        self.last_report_wall = 0.0
        self.last_temp_wall = 0.0
        # worst sim-time gap between consecutive DISTINCT plane positions
        # actually bridged to the copters, measured on the plane's own
        # clock; large values mean FOLLOW chased a stale target
        self.last_bridge_boot_ms = None
        self.max_bridge_gap_sim_s = 0.0
        self.last_speed_sample = None
        # (wall_s, sim_ms) pairs for the end-of-run speedup verdict
        self.speed_samples = []

    def drain_all(self):
        '''read every link, so no vehicle blocks on a full send buffer'''
        for v in self.vehicles:
            if v.mav is not None:
                v.drain()

    def pump(self, until, timeout, desc, bridge=True):
        '''run the bridge and telemetry until until() returns True'''
        self.phase = desc
        dl = Deadline(timeout, desc)

        while True:
            for v in self.vehicles:
                if not v.alive():
                    raise TestFailure("%s exited during '%s'" % (v.name, desc))
                v.drain()

            if bridge:
                self.bridge()
            self.report()

            if until():
                return dl.elapsed()

            dl.check()
            time.sleep(self.opts.loop_dt)

    def bridge(self):
        '''forward the plane's position onto each copter's link'''
        pmsg = self.plane.mav.messages.get("GLOBAL_POSITION_INT", None)
        if pmsg is None:
            return
        if (self.last_bridge_boot_ms is not None and
                pmsg.time_boot_ms > self.last_bridge_boot_ms):
            gap = (pmsg.time_boot_ms - self.last_bridge_boot_ms) / 1000.0
            self.max_bridge_gap_sim_s = max(self.max_bridge_gap_sim_s, gap)
        self.last_bridge_boot_ms = pmsg.time_boot_ms
        for c in self.copters:
            # preserve the plane's sysid in the header - that is what
            # AP_Follow matches against FOLL_SYSID
            c.mav.mav.srcSystem = self.plane.sysid
            c.mav.mav.global_position_int_send(
                pmsg.time_boot_ms, pmsg.lat, pmsg.lon, pmsg.alt,
                pmsg.relative_alt, pmsg.vx, pmsg.vy, pmsg.vz, pmsg.hdg)
            c.mav.mav.srcSystem = 250

    def report(self):
        wall_s = time.time()
        if wall_s - self.last_report_wall < self.opts.report_interval:
            return
        self.last_report_wall = wall_s
        # live achieved speedup from the plane's own clock: time_boot_ms
        # is simulation-paced, the report interval is wall-paced, so
        # their ratio is the speedup actually achieved right now
        speed = ""
        pmsg = self.plane.mav.messages.get("GLOBAL_POSITION_INT", None)
        if pmsg is not None:
            sim_ms = pmsg.time_boot_ms
            if self.last_speed_sample is not None:
                prev_sim_ms, prev_wall = self.last_speed_sample
                d_wall = wall_s - prev_wall
                if d_wall > 0 and sim_ms > prev_sim_ms:
                    speed = " speedup=%.1fx" % (
                        (sim_ms - prev_sim_ms) / 1000.0 / d_wall)
            self.last_speed_sample = (sim_ms, wall_s)
            self.speed_samples.append((wall_s, sim_ms))
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
                parts.append("%s err=%.0fm" % (
                    c.name, horizontal_distance(want, cpos)))
            dists = " " + " ".join(parts)
        progress("[%s] t=%.0fs%s%s" % (self.phase, wall_s - self.t0, speed, dists))

    def run(self):
        # Connect to each vehicle the moment it is launched. SITL's SERIAL0
        # defaults to "tcp:0:wait", which blocks the physics loop until a
        # client attaches, so a vehicle that is started but not yet connected
        # is not simulating at all - it just sits at "Waiting for connection".
        # connect() retries until the port is listening, so no fixed sleep is
        # needed here.
        for v in self.vehicles:
            progress("starting %s (instance %u, sysid %u) at %ux speedup"
                     % (v.name, v.instance, v.sysid, v.speedup))
            v.start()
            v.connect()
            progress("%s connected on port %u" % (v.name, v.port))

        # Whether they clustered is AP_SITL_SharedMem's business, not
        # Python's: the vehicles print "joined cluster" and "in lock-step
        # with N instances" to this console themselves.
        for v in self.vehicles:
            v.drain_hook = self.drain_all

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
        self.pump(lambda: self.plane.rel_alt() > 80,
                  timeout=300, desc="plane-takeoff", bridge=False)
        progress("plane airborne at %.0fm, switching to LOITER"
                 % self.plane.rel_alt())
        self.plane.set_mode("LOITER")

        progress("launching copters")
        for c in self.copters:
            c.set_mode("GUIDED")
            c.arm()
            c.takeoff_cmd(60)

        # a single NAV_TAKEOFF can be lost or refused on a raced link
        # (seen on the Cygwin CI runner: both copters verified armed in
        # GUIDED yet sat on the ground for the entire phase). Re-send
        # every few wall seconds until each copter is demonstrably
        # climbing.
        def takeoff_watch():
            now = time.time()
            for c in self.copters:
                if (c.rel_alt() < 2.0 and
                        now - getattr(c, "last_takeoff_resend", 0) > 5.0):
                    c.last_takeoff_resend = now
                    c.takeoff_cmd(60)
            return all(c.rel_alt() > 50 for c in self.copters)

        self.pump(takeoff_watch, timeout=300, desc="copter-takeoff")
        progress("copters airborne at %s"
                 % ["%.0fm" % c.rel_alt() for c in self.copters])

        for c in self.copters:
            c.set_mode("FOLLOW")
        progress("all copters in FOLLOW, tracking for %.0f seconds"
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
        last = {"t": None}

        def track():
            t = time.time()
            dt = 0.0 if last["t"] is None else t - last["t"]
            last["t"] = t
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
            self.pump(track, timeout=self.opts.follow_time, desc="follow")
        except TestFailure as e:
            if "timed out after" not in str(e):
                raise           # a real failure, not the intended duration

        # surface each vehicle's governor decisions and its own achieved
        # speedup from the log files (vehicle output no longer reaches
        # this console)
        for v in self.vehicles:
            try:
                f = open(os.path.join(self.logdir, "%s.log" % v.name), "rb")
                lines = [ln.decode("utf-8", "replace").strip()
                         for ln in f if b"adaptive rate" in ln or b"achieved" in ln]
                f.close()
            except (OSError, IOError):
                lines = []
            gov = [ln for ln in lines if "adaptive rate" in ln]
            ach = [ln for ln in lines if "adaptive rate" not in ln]
            for ln in gov[-2:]:
                progress("%s governor: %s" % (v.name, ln))
            for ln in ach[-3:]:
                progress("%s: %s" % (v.name, ln))

        progress("worst gap between position updates: "
                 "%.2f sim-seconds (FOLL_TIMEOUT is 30s)"
                 % self.max_bridge_gap_sim_s)
        for c in self.copters:
            progress("%s: best formation error %s, held station %.0f seconds unbroken"
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
                    "%s only held formation for %.0f seconds unbroken, "
                    "wanted %.0f"
                    % (c.name, settled[c.name], self.opts.min_hold_time))

        # steady-state speedup over the back half of the follow phase,
        # and the hard verdict: any shortfall from the commanded
        # speedup is a failure - the adaptive governor is required to
        # slow the physics until the commanded speedup is reached
        steady = None
        if len(self.speed_samples) >= 4:
            last_wall, last_sim = self.speed_samples[-1]
            cutoff = last_wall - self.opts.follow_time / 2.0
            back = [p for p in self.speed_samples if p[0] >= cutoff]
            if len(back) >= 2 and back[-1][0] > back[0][0]:
                steady = ((back[-1][1] - back[0][1]) / 1000.0
                          / (back[-1][0] - back[0][0]))
                progress("steady-state speedup (back half of follow): "
                         "%.1fx" % steady)
        if steady is not None and steady < self.opts.speedup:
            raise TestFailure(
                "achieved steady-state speedup %.1fx is below the "
                "commanded %ux" % (steady, self.opts.speedup))

        progress("PASS: %u copters followed the plane in cluster %u"
                 % (len(self.copters), self.opts.cluster))

    def cleanup(self):
        for v in self.vehicles:
            v.stop()


class Settings(object):
    """all knobs live here - edit the values, there are no command-line
    arguments"""

    copter_binary = "./ArduCopter.elf.exe"
    plane_binary = "./ArduPlane.elf.exe"

    cluster = 91            # cluster id the vehicles join
    # requested SITL speedup for every vehicle. CI overrides via the
    # CLUSTER_SPEEDUP environment (the Windows runners sustain ~55-65x,
    # so their smoke commands 50x and the achieved-vs-commanded gate
    # stays strict; the Linux CI commands and delivers 100x)
    speedup = float(os.environ.get("CLUSTER_SPEEDUP", "100"))

    # bridge/telemetry loop period, WALL seconds. speedup * loop_dt is the
    # sim-time gap between position updates the copters see, and must stay
    # well under FOLL_TIMEOUT_S or the copters drop the plane.
    loop_dt = 0.01

    # seconds to hold formation for. The default suits a quick local
    # run; CI sets CLUSTER_FOLLOW_TIME so the adaptive governor's full
    # descent (2-3 minutes) fits inside the measured window.
    # sized so the WHOLE run (boot, arm, takeoff, follow, verdict)
    # fits under 60 wall seconds
    follow_time = float(os.environ.get("CLUSTER_FOLLOW_TIME", "25"))
    report_interval = 2.0   # seconds between progress lines

    # metres of formation error that still counts as holding station.
    # Wider than the Linux harness's 250m: this smoke gates the cluster
    # MECHANISM under Cygwin (lock-step, FOLLOW engaging, vehicles
    # flying), while formation precision is gated by the Linux CI. The
    # harness position bridge adds ~100m of target staleness at high
    # speedup on the slower Windows runners (see the worst-gap line in
    # the verdict), which 250m sliced straight through.
    hold_distance = 400.0
    # seconds each copter must hold formation for WITHOUT a break,
    # clamped under follow_time (CI shortens the window via
    # CLUSTER_FOLLOW_TIME, and a hold requirement longer than the
    # window is impossible to satisfy)
    min_hold_time = min(30.0, follow_time * 0.8)


def main():
    opts = Settings()

    # resolve now: each SITL is spawned with cwd set to its log directory,
    # so a relative binary path would be looked up in the wrong place
    opts.copter_binary = os.path.abspath(opts.copter_binary)
    opts.plane_binary = os.path.abspath(opts.plane_binary)
    for path in (opts.copter_binary, opts.plane_binary):
        if not os.path.exists(path):
            print("FAIL: missing binary %s - run this script from the "
                  "directory holding the SITL binaries" % path)
            return 1

    power_previous = set_windows_power_performance()
    disable_power_throttling([opts.copter_binary, opts.plane_binary])

    test = ClusterFollowTest(opts)
    try:
        test.run()
        return 0
    except TestFailure as e:
        progress("FAIL: %s" % e)
        return 1
    except KeyboardInterrupt:
        progress("interrupted")
        return 1
    finally:
        test.cleanup()
        restore_windows_power(power_previous)


if __name__ == "__main__":
    sys.exit(main())
