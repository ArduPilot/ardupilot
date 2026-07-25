#!/usr/bin/env python3
"""
ArduPilot SITL <-> Webots bridge for the UGV rover (Ackermann: front steer +
rear drive).

Protocol (matches ArduPilot's SIM_Webots_Python backend, model "webots-python"):
  - ArduPilot -> here: 16 float32 servo outputs in [0,1] (-1 = channel unused).
        SERVO1 (index 0) = steering, SERVO3 (index 2) = throttle (bidirectional).
  - here -> ArduPilot: 16 float64 FDM = timestamp, gyro[3], accel[3],
        imu_rpy[3], velocity[3], position[3], with an ENU->NED axis flip
        (negate Y and Z), exactly as ArduPilot's reference webots_vehicle.py.

The rover's own C controller (ugv_teleop) is for manual/autonomous driving;
this separate controller hands control to ArduPilot instead. Select it on the
rover's `controller` field (e.g. use the ardupilot_rover.wbt world).

Networking (WSL2 in NAT mode does NOT share localhost):
  * Set SITL_ADDRESS below to the WSL IP where SITL runs (`hostname -I`).
  * Launch SITL with `--sim-address <windows-host-ip>` (from WSL:
    `ip route show default`), so its control packets reach this bridge.
  * Windows Firewall must allow inbound UDP on PORT from the WSL subnet.
"""

import os
import sys
import math
import socket
import select
import struct

# --- Webots controller module (also lets this run as an extern controller) ---
if os.environ.get("WEBOTS_HOME") is None:
    os.environ["WEBOTS_HOME"] = r"C:\Program Files\Webots"
sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Robot  # noqa: E402

# ------------------------- configuration -------------------------
SITL_ADDRESS = os.environ.get("SITL_ADDRESS", "172.21.7.109")  # WSL IP (where SITL runs)
PORT = 9002  # bind here for controls; FDM is sent to SITL_ADDRESS:PORT+1

# Must match the rover model / world.
MAX_STEER_ANGLE = 0.70    # rad (~40 deg), front-steer joint limit -> ~1.42 m turn radius
MAX_WHEEL_SPEED = 20.0    # rad/s, rear drive-motor maxVelocity
WHEELBASE = 1.20
TRACK_WIDTH = 1.10

# Flip these to +1/-1 if a channel drives the wrong way (depends on
# SERVOx_REVERSED in ArduPilot). Verified by test: with STEER_SIGN=+1 the nav
# loop diverged on every turn (ArduPilot commanded left, the wheels turned
# right, heading error grew) while straight-line drove correctly - i.e. the
# steering direction was inverted. STEER_SIGN=-1 corrects it. DRIVE_SIGN stays
# +1 (forward/back drove the right way).
STEER_SIGN = -1.0
DRIVE_SIGN = 1.0

CONTROLS_FMT = "f" * 16          # from ArduPilot
CONTROLS_SIZE = struct.calcsize(CONTROLS_FMT)
FDM_FMT = "d" * (1 + 3 + 3 + 3 + 3 + 3)  # to ArduPilot


# ------------------------- Webots setup -------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

steer_fl = robot.getDevice("steer_fl")
steer_fr = robot.getDevice("steer_fr")
wheel_rl = robot.getDevice("wheel_rl")
wheel_rr = robot.getDevice("wheel_rr")

# rear wheels: velocity control
for w in (wheel_rl, wheel_rr):
    w.setPosition(float("inf"))
    w.setVelocity(0.0)
# front wheels: position (steering) control
steer_fl.setPosition(0.0)
steer_fr.setPosition(0.0)

gps = robot.getDevice("gps")
imu = robot.getDevice("imu")
gyro = robot.getDevice("gyro")
accel = robot.getDevice("accel")
for s in (gps, imu, gyro, accel):
    s.enable(timestep)

# LD2451-style corner radars (front pair aimed forward, rear pair aft). The
# nearest front/rear target (distance + radial speed) is streamed to a WSL
# companion (tools/radar_mav.py), which re-encodes it as real HLK-LD2451 UART
# frames feeding ArduPilot's AP_Proximity_HLK_LD2451 backend directly (not a
# generic MAVLink DISTANCE_SENSOR). getTargets() returns moving objects that
# have a radarCrossSection (e.g. the "target_car") - the real sensor is
# motion-only too, so stationary geometry never appears here.
RADAR_OUT_PORT = 9005            # UDP port the companion listens on (in WSL)
front_radars, rear_radars = [], []
for rname in ("radar_fl", "radar_fr", "radar_rl", "radar_rr"):
    dev = robot.getDevice(rname)
    if dev is not None:
        dev.enable(timestep)
        (front_radars if rname.startswith("radar_f") else rear_radars).append((rname, dev))
radars = front_radars + rear_radars

_radar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_next_radar_log = 0.0
_next_prox_send = 0.0


def _nearest(group):
    """Closest target (Webots Target object) across a group of radars, or None."""
    best = None
    for _, dev in group:
        for t in dev.getTargets():
            if best is None or t.distance < best.distance:
                best = t
    return best


def send_proximity(sitl_ip):
    """Stream nearest front/rear target (distance_m, |radial speed| m/s) to the
    companion (~10 Hz). -1 distance means "no target" (clear); speed is 0 in
    that case. Speed is sent unsigned - the companion derives approach/depart
    direction itself from the distance trend, since Webots' Target.speed sign
    convention isn't documented."""
    global _next_prox_send
    now = robot.getTime()
    if now < _next_prox_send:
        return
    _next_prox_send = now + 0.1
    front = _nearest(front_radars)
    rear = _nearest(rear_radars)
    fd, fs = (front.distance, abs(front.speed)) if front is not None else (-1.0, 0.0)
    rd, rs = (rear.distance, abs(rear.speed)) if rear is not None else (-1.0, 0.0)
    msg = f"{fd:.2f},{fs:.2f},{rd:.2f},{rs:.2f}"
    try:
        _radar_sock.sendto(msg.encode(), (sitl_ip, RADAR_OUT_PORT))
    except OSError:
        pass


def report_radar():
    """Print any radar targets, throttled to ~2 Hz (distance/azimuth/speed)."""
    global _next_radar_log
    now = robot.getTime()
    if now < _next_radar_log:
        return
    _next_radar_log = now + 0.5
    hits = []
    for rname, dev in radars:
        for t in dev.getTargets():
            hits.append(f"{rname}: {t.distance:.1f} m  "
                        f"{math.degrees(t.azimuth):+.0f} deg  {t.speed:+.1f} m/s")
    if hits:
        print("[radar] " + " | ".join(hits))
        sys.stdout.flush()


def ackermann(speed, steer_angle):
    """Rear left/right wheel speeds for a given centre speed and steer angle."""
    if abs(steer_angle) < 0.01:
        return speed, speed
    turn_radius = WHEELBASE / math.tan(steer_angle)
    left = speed * (turn_radius - TRACK_WIDTH / 2.0) / turn_radius
    right = speed * (turn_radius + TRACK_WIDTH / 2.0) / turn_radius
    return left, right


_dbg = {}  # last controls/commands, for the debug log


def handle_controls(cmd):
    """Apply ArduPilot servo outputs (each 0..1, 0.5 = centre/stop)."""
    steer_norm = cmd[0]   # SERVO1 = steering
    throttle_norm = cmd[2]  # SERVO3 = throttle
    # -1 marks an unused channel; treat as neutral
    if steer_norm < 0.0:
        steer_norm = 0.5
    if throttle_norm < 0.0:
        throttle_norm = 0.5

    steer_angle = STEER_SIGN * (steer_norm - 0.5) * 2.0 * MAX_STEER_ANGLE
    speed = DRIVE_SIGN * (throttle_norm - 0.5) * 2.0 * MAX_WHEEL_SPEED

    steer_fl.setPosition(steer_angle)
    steer_fr.setPosition(steer_angle)
    left, right = ackermann(speed, steer_angle)
    wheel_rl.setVelocity(left)
    wheel_rr.setVelocity(right)

    _dbg.update(c0=cmd[0], c1=cmd[1], c2=cmd[2], c3=cmd[3],
                steer=steer_angle, rl=left, rr=right)


def get_fdm():
    """Pack the Flight Dynamics Model (sensor state) for ArduPilot.

    Proper Webots-ENU -> ArduPilot-NED conversion so ArduPilot's position and
    heading match the real (OSM) map, i.e. it uses the Webots GPS/heading
    directly:
      Webots world ENU: X=East, Y=North, Z=Up.
      ArduPilot NED:    North=Y, East=X, Down=-Z.
      Heading: the InertialUnit yaw is referenced from +X (East), so the NED
               heading (from North) is pi/2 - yaw_enu.
      Gyro/accel are body-frame (FLU->FRD: X same, negate Y,Z) - frame-agnostic.
    """
    i = imu.getRollPitchYaw()
    g = gyro.getValues()
    a = accel.getValues()
    p = gps.getValues()
    v = gps.getSpeedVector()
    yaw_ned = math.pi / 2.0 - i[2]
    return struct.pack(
        FDM_FMT,
        robot.getTime(),
        g[0], -g[1], -g[2],           # gyro (body FRD)
        a[0], -a[1], -a[2],           # accel (body FRD)
        i[0], -i[1], yaw_ned,         # roll, pitch, yaw (NED)
        v[1], v[0], -v[2],            # velocity NED (North=Y, East=X, Down=-Z)
        p[1], p[0], -p[2],            # position NED (North=Y, East=X, Down=-Z)
    )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.setblocking(False)

    print(f"ardupilot_bridge: listening for SITL controls on 0.0.0.0:{PORT}")
    sys.stdout.flush()

    # SITL's address is learned from the first control packet (so we don't have
    # to hard-code the WSL IP, which changes across reboots). FDM is then sent
    # back to that IP on PORT+1. SITL_ADDRESS above is only a documented default.
    sitl_addr = None

    # Wait for SITL to start sending, stepping Webots so the GUI stays alive.
    # Radar still reports here, so the sensor demo works even without SITL.
    while not select.select([sock], [], [], 0)[0]:
        if robot.step(timestep) == -1:
            sock.close()
            return
        report_radar()
        send_proximity(sitl_addr[0] if sitl_addr else SITL_ADDRESS)

    # Debug log (written next to this controller so it can be inspected).
    log = open("bridge_log.csv", "w")
    log.write("time,c0_steer,c1,c2_thr,c3,steer_rad,rl_vel,rr_vel,gps_x,gps_y,gps_yaw_deg\n")
    next_log = 0.0
    print("GPS Yaw: {:.1f} deg".format(imu.getRollPitchYaw()[2] * 180.0 / math.pi))

    # Main loop: exactly ONE Webots step per control packet (lockstep with
    # SITL). Sending FDM whenever the socket is writable keeps SITL fed. An
    # earlier version stepped twice per packet, which halved the sensor/loop
    # rate and desynced ArduPilot's EKF - do not reintroduce a second step().
    while True:
        readable, writable, _ = select.select([sock], [sock], [], 0)

        if readable:
            data, addr = sock.recvfrom(512)
            if data and len(data) >= CONTROLS_SIZE:
                if sitl_addr is None:
                    sitl_addr = (addr[0], PORT + 1)
                    print(f"ardupilot_bridge: connected to ArduPilot SITL at {addr[0]}")
                    sys.stdout.flush()
                cmd = struct.unpack(CONTROLS_FMT, data[:CONTROLS_SIZE])
                handle_controls(cmd)
                if robot.step(timestep) == -1:
                    break
                report_radar()
                send_proximity(sitl_addr[0] if sitl_addr else SITL_ADDRESS)

                now = robot.getTime()
                if now >= next_log and _dbg:
                    next_log = now + 0.5
                    p = gps.getValues()
                    yaw = imu.getRollPitchYaw()[2] * 180.0 / math.pi
                    log.write(f"{now:.2f},{_dbg['c0']:.4f},{_dbg['c1']:.4f},{_dbg['c2']:.4f},"
                              f"{_dbg['c3']:.4f},{_dbg['steer']:.4f},{_dbg['rl']:.3f},{_dbg['rr']:.3f},"
                              f"{p[0]:.3f},{p[1]:.3f},{yaw:.1f}\n")
                    log.flush()
                    # print("GPS Yaw: {:.1f} deg".format(imu.getRollPitchYaw()[2] * 180.0 / math.pi))

        if writable and sitl_addr is not None:
            sock.sendto(get_fdm(), sitl_addr)

    log.close()
    sock.close()
    print("ardupilot_bridge: Webots closed, exiting")


if __name__ == "__main__":
    main()
