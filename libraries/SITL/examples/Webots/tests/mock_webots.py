#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
A stand-in for the Webots simulator, so SIM_Webots.cpp can be exercised on a
machine with no Webots installed, with a rigid-body
quad whose rotors follow Webots' Propeller semantics:

    thrust_i = kT * |omega_i| * omega_i        (per cyberbotics.com/doc/reference/propeller)
    torque_i = kQ * |omega_i| * omega_i

--proto json-tcp     SIM_Webots.cpp, the C/C++ driver in this directory
--proto struct-udp   SIM_Webots_Python.cpp, for cross-checking

--profile quadx          the recalibrated webots_quadX.wbt
--profile quadx-legacy   the constants that world shipped with before
"""

import argparse
import math
import select
import socket
import struct
import sys
import time

import numpy as np

G = 9.80665

# Composing each arm's Pose and Solid transforms in webots_quadX.wbt puts the
# rotors at 0.4 m from the centre; the -45 deg sensor mount makes that an X in
# the body frame.  Inertia is 0.1 m, 1.0 kg body box plus 4 x 0.05 kg nacelles
# at that radius (QUADCOPTER_TUNING.md section 2).  Both profiles are the same
# airframe, only the rotor constants differ.
ARM_RADIUS = 0.4
_D = ARM_RADIUS / math.sqrt(2)
ROTOR_POS = [(+_D, +_D), (-_D, -_D), (+_D, -_D), (-_D, +_D)]
INERTIA = [0.0178, 0.0178, 0.0338]

PROFILES = {
    # webots_quadX.wbt / webots_quadPlus.wbt, as written by migrate_worlds.py
    'quadx': dict(
        mass=1.2,                           # 1.0 kg body + 4 x 0.05 kg rotor nacelles
        inertia=INERTIA,                    # see QUADCOPTER_TUNING.md section 2
        kT=4.04524e-05, kQ=8.09049e-07,     # thrustConstants / torqueConstants
        omega_max=400.0,                    # RotationalMotor maxVelocity, rad/s
        linearize=True,                     # controller commands sqrt(u) * omega_max
        motor_accel=5000.0,                 # rad/s^2: the world's maxTorque (see below)
        dt=0.001,                           # WorldInfo basicTimeStep
        pos=ROTOR_POS,
    ),
    # the constants these worlds shipped with before the recalibration, kept so
    # the regression can show what changed
    'quadx-legacy': dict(
        mass=1.0,
        inertia=INERTIA,
        kT=1.00001, kQ=1.1,
        omega_max=3.6,                      # old controller's factorDyn[0]
        linearize=False,                    # factorDyn[10 * (int)u] never varied
        motor_accel=90.0,                   # those worlds' maxTorque 90
        dt=0.001,
        pos=ROTOR_POS,
    ),
}

MOTOR_DIR = np.array([+1.0, +1.0, -1.0, -1.0])
DRAG = 0.10


def dcm_to_euler(R):
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


class Quad:
    def __init__(self, p):
        self.p = p
        self.mass = p['mass']
        self.inertia = np.diag(p['inertia'])
        self.inertia_inv = np.linalg.inv(self.inertia)
        self.mpos = np.array(p['pos'])
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.R = np.eye(3)
        self.omega = np.zeros(3)
        self.rotor = np.zeros(4)
        self.cmd_rotor = np.zeros(4)
        self.accel_body = np.array([0.0, 0.0, -G])
        self.t = 0.0
        self.sat_hi = 0
        self.sat_lo = 0
        self.nsamp = 0

    def set_u(self, u):
        u = np.clip(np.asarray(u[:4], dtype=float), 0.0, 1.0)
        self.u = u
        w = np.sqrt(u) if self.p['linearize'] else u
        self.cmd_rotor = w * self.p['omega_max']

    def step(self, dt):
        # measured in Webots R2025a: a Propeller's shaft slews at a constant
        # maxTorque rad/s^2 towards its commanded speed
        step = self.p['motor_accel'] * dt
        if step > 0:
            self.rotor += np.clip(self.cmd_rotor - self.rotor, -step, step)
        else:
            self.rotor = self.cmd_rotor

        thrust = self.p['kT'] * np.abs(self.rotor) * self.rotor
        qtor = self.p['kQ'] * np.abs(self.rotor) * self.rotor

        f_body = np.array([0.0, 0.0, -thrust.sum()])
        moment = np.array([
            -float(np.dot(self.mpos[:, 1], thrust)),
            +float(np.dot(self.mpos[:, 0], thrust)),
            float(np.dot(MOTOR_DIR, qtor)),
        ])

        v_body = self.R.T @ self.vel
        f_body = f_body - DRAG * v_body * np.abs(v_body)

        if self.pos[2] >= -1e-3 and f_body[2] > -self.mass * G:
            self.pos[2] = 0.0
            self.vel[:] = 0.0
            self.omega[:] = 0.0
            self.accel_body = self.R.T @ np.array([0.0, 0.0, -G])
            self.t += dt
            return

        self.accel_body = f_body / self.mass
        self.vel += (self.R @ self.accel_body + np.array([0.0, 0.0, G])) * dt
        self.pos += self.vel * dt

        wdot = self.inertia_inv @ (moment - np.cross(self.omega, self.inertia @ self.omega))
        self.omega += wdot * dt
        w = self.omega * dt
        skew = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        u_, _, vt = np.linalg.svd(self.R @ (np.eye(3) + skew))
        self.R = u_ @ vt
        self.t += dt


# ------------------------------------------------------------------ protocols
def sensor_json(q, rpm=True):
    roll, pitch, yaw = dcm_to_euler(q.R)
    p, qq, r = q.omega
    ax, ay, az = q.accel_body
    n, e, d = q.pos
    vn, ve, vd = q.vel
    # rotor i is driven by SITL servo channel i, which is the order SIM_Webots
    # and AP_RPM expect the "rpm" array in
    rpm_json = (',"rpm": [%.1f, %.1f, %.1f, %.1f]' % tuple(q.rotor * 60.0 / (2 * math.pi))
                if rpm else '')
    # the same framing as controllers/common/sensors.c getAllSensors()
    return ('{"ts": %.6f,'
            '"vehicle.imu": {"av": [%f, %f, %f],"la": [%f, %f, %f],'
            '"mf": [23088.0, 3875.0, -53204.0]},'
            '"vehicle.gps": {"x": %f,"y": %f,"z": %f},'
            '"vehicle.velocity": {"wlv": [%f, %f, %f]},'
            '"vehicle.pose": {"x": %f,"y": %f,"z": %f,"roll": %f,"pitch": %f,"yaw": %f}'
            '%s}\n'
            % (q.t, p, qq, -r, ax, ay, -az, n, e, -d, vn, ve, -vd,
               n, e, -d, roll, pitch, -yaw, rpm_json)).encode()


FDM_FMT = 'd' * 16


def fdm_struct(q):
    roll, pitch, yaw = dcm_to_euler(q.R)
    return struct.pack(FDM_FMT, q.t,
                       q.omega[0], q.omega[1], q.omega[2],
                       q.accel_body[0], q.accel_body[1], q.accel_body[2],
                       roll, pitch, yaw,
                       q.vel[0], q.vel[1], q.vel[2],
                       q.pos[0], q.pos[1], q.pos[2])


def parse_pwm_json(line):
    a = line.find('"pwm"')
    if a < 0:
        return None
    lb = line.find('[', a)
    rb = line.find(']', lb)
    try:
        return [(float(x) - 1000.0) / 1000.0 for x in line[lb + 1:rb].split(',')]
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--proto', choices=['json-tcp', 'struct-udp'], default='json-tcp')
    ap.add_argument('--profile', choices=list(PROFILES), default='quadx')
    ap.add_argument('--port', type=int, default=5577)
    ap.add_argument('--dt', type=float, default=None)
    ap.add_argument('--motor-accel', type=float, default=None,
                    help='rotor slew rate, rad/s^2 (0 = instant)')
    ap.add_argument('--no-rpm', action='store_true',
                    help='leave out the "rpm" key, like an older controller')
    ap.add_argument('--log', default=None)
    ap.add_argument('--stats', default=None)
    ap.add_argument('--lockstep', action='store_true',
                    help='block for the servo frame instead of re-sending sensors')
    ap.add_argument('--max-speed', type=float, default=0.0,
                    help='cap sim seconds per wall second (0 = unlimited)')
    args = ap.parse_args()

    prof = dict(PROFILES[args.profile])
    if args.motor_accel is not None:
        prof['motor_accel'] = args.motor_accel
    dt = args.dt if args.dt is not None else prof['dt']
    q = Quad(prof)
    q.u = np.zeros(4)

    hover_u = None
    print('mock: proto=%s profile=%s dt=%.4f  kQ/kT=%.3f m  omega_hover=%.1f rad/s (%.0f rpm)'
          % (args.proto, args.profile, dt, prof['kQ'] / prof['kT'],
             math.sqrt(prof['mass'] * G / 4 / prof['kT']),
             math.sqrt(prof['mass'] * G / 4 / prof['kT']) * 60 / (2 * math.pi)), flush=True)

    logf = open(args.log, 'w') if args.log else None
    if logf:
        logf.write('t,n,e,alt,roll,pitch,yaw,u0,u1,u2,u3,w0,w1,w2,w3\n')

    if args.proto == 'json-tcp':
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('127.0.0.1', args.port))
        srv.listen(1)
        print('mock: TCP listening on %d' % args.port, flush=True)
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print('mock: SITL connected', flush=True)
        sock = conn
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', args.port))
        print('mock: UDP bound %d, FDM -> %d' % (args.port, args.port + 1), flush=True)

    rx = b''
    steps = 0
    wall0 = time.time()
    try:
        while True:
            if args.proto == 'json-tcp':
                sock.sendall(sensor_json(q, rpm=not args.no_rpm))
            else:
                sock.sendto(fdm_struct(q), ('127.0.0.1', args.port + 1))

            u = None
            if select.select([sock], [], [], 5.0 if args.lockstep else 0.0)[0]:
                data = sock.recv(4096)
                if args.proto == 'json-tcp':
                    if not data:
                        print('mock: SITL closed', flush=True)
                        break
                    rx += data
                    if b'\n' in rx:
                        parts = rx.split(b'\n')
                        rx = parts[-1]
                        for cand in reversed(parts[:-1]):
                            u = parse_pwm_json(cand.decode('utf-8', 'replace'))
                            if u:
                                break
                else:
                    if len(data) >= 64:
                        u = list(struct.unpack('f' * 16, data[:64]))
            if u is None:
                time.sleep(0.0002)
                continue

            q.set_u(u)
            q.step(dt)
            steps += 1

            alt = -q.pos[2]
            if alt > 5.0:
                q.nsamp += 1
                q.sat_hi += int((q.u > 0.98).any())
                q.sat_lo += int((q.u < 0.02).any())
                if hover_u is None:
                    hover_u = []
                hover_u.append(float(q.u.mean()))

            if logf and steps % 10 == 0:
                r_, p_, y_ = dcm_to_euler(q.R)
                logf.write('%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f\n'
                           % (q.t, q.pos[0], q.pos[1], alt, r_, p_, y_,
                              *q.u, *q.rotor))

            if args.max_speed > 0:
                behind = q.t / args.max_speed - (time.time() - wall0)
                if behind > 0.0005:
                    time.sleep(behind)

            if steps % 3000 == 0:
                print('mock: t=%.1f alt=%.2f rpy=%.1f/%.1f/%.1f u=%s rotor=%s wall=%.0fs'
                      % (q.t, alt, *[math.degrees(x) for x in dcm_to_euler(q.R)],
                         np.round(q.u, 3), np.round(q.rotor, 1), time.time() - wall0),
                      flush=True)
    except (BrokenPipeError, ConnectionResetError, OSError) as e:
        print('mock: %s' % e, flush=True)
    finally:
        if logf:
            logf.close()
        if args.stats and q.nsamp:
            with open(args.stats, 'w') as f:
                f.write('profile=%s proto=%s\n' % (args.profile, args.proto))
                f.write('hover_throttle_mean=%.4f\n' % (sum(hover_u) / len(hover_u)))
                f.write('samples_above_5m=%d\n' % q.nsamp)
                f.write('pct_steps_with_a_motor_at_max=%.2f\n' % (100.0 * q.sat_hi / q.nsamp))
                f.write('pct_steps_with_a_motor_at_min=%.2f\n' % (100.0 * q.sat_lo / q.nsamp))
        sock.close()


if __name__ == '__main__':
    sys.exit(main())
