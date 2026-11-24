"""Software-only 6-DoF underwater dynamics simulator for native ArduSub testing.

Consumes localhost RCOutput_ATAL_Loopback frames (never physical hardware),
computes generalized body wrench from vehicle_defaults.json motor geometry,
integrates damped 6-DoF rigid-body kinematics, and emits consistent Xsens
MTData2, Bar30 DEPTH:, and Water Linked A50 JSON reports.
"""

import json
import math
from pathlib import Path
import struct


GRAVITY_MPS2 = 9.81
ATM_MBAR = 1013.25
WATER_DENSITY = 1000.0


def euler_to_quat_wxyz(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def rotate_body_to_ned(roll, pitch, yaw, vec):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = vec
    r00, r01, r02 = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    r10, r11, r12 = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    r20, r21, r22 = -sp, cp * sr, cp * cr
    return (
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    )


def rotate_ned_to_body(roll, pitch, yaw, vec):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = vec
    r00, r01, r02 = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    r10, r11, r12 = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    r20, r21, r22 = -sp, cp * sr, cp * cr
    return (
        r00 * x + r10 * y + r20 * z,
        r01 * x + r11 * y + r21 * z,
        r02 * x + r12 * y + r22 * z,
    )


class UnderwaterDynamicsSim:
    """Damped 6-DoF rigid-body vehicle model for ArduSub software verification."""

    def __init__(self, config_path=None):
        cfg_file = Path(config_path) if config_path else (Path(__file__).with_name("vehicle_defaults.json"))
        cfg = json.loads(cfg_file.read_text(encoding="utf-8"))
        self.pwm_min = cfg["actuators"]["pwm_min_us"]
        self.pwm_neutral = cfg["actuators"]["pwm_neutral_us"]
        self.pwm_max = cfg["actuators"]["pwm_max_us"]
        self.motors = cfg["actuators"]["motors"]
        # Build 8x6 motor factor rows indexed by AP servo channel 1..8 directly from AP_Motors6DOF.cpp
        from check_layout import MOTORS, upstream_factors
        upstream = upstream_factors(MOTORS.read_text(encoding="utf-8"))
        self.ap_factors = [list(upstream[i]) for i in range(1, 9)]
        self.reset()

    def reset(self):
        self.pos_ned = [0.0, 0.0, 0.5]  # start 0.5m below surface
        self.vel_body = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.rates_body = [0.0, 0.0, 0.0]
        self.accel_body = [0.0, 0.0, -GRAVITY_MPS2]

    def step(self, pwm_channels_1_to_8, armed, dt_s):
        """Advance vehicle state by dt_s using AP servo PWM channels 1..8."""
        span = max(1.0, 0.5 * (self.pwm_max - self.pwm_min))
        wrench = [0.0] * 6  # roll, pitch, yaw, vertical(up+), forward, lateral
        if armed:
            for idx in range(8):
                pwm = float(pwm_channels_1_to_8[idx])
                # MOT_n_DIRECTION = -1 flips normalized servo around neutral
                thrust = max(-1.0, min(1.0, -(pwm - self.pwm_neutral) / span))
                row = self.ap_factors[idx]
                for ax in range(6):
                    wrench[ax] += row[ax] * thrust

        # Wrench scales (nominal pool dynamics for controller/allocation validation)
        tau_r, tau_p, tau_y, f_up, f_fwd, f_lat = (
            wrench[0] * 1.5, wrench[1] * 1.5, wrench[2] * 1.8,
            wrench[3] * 12.0, wrench[4] * 12.0, wrench[5] * 12.0,
        )
        # In NED body frame, vertical+ is up -> downward force is -f_up
        lin_accel_body = [
            f_fwd / 12.0 - 2.2 * self.vel_body[0],
            f_lat / 12.0 - 2.2 * self.vel_body[1],
            -f_up / 12.0 - 2.5 * self.vel_body[2],
        ]
        # Restoring righting moment in roll/pitch + rotational damping
        ang_accel_body = [
            tau_r / 0.35 - 3.5 * self.rates_body[0] - 4.0 * math.sin(self.rpy[0]),
            tau_p / 0.35 - 3.5 * self.rates_body[1] - 4.0 * math.sin(self.rpy[1]),
            tau_y / 0.45 - 3.0 * self.rates_body[2],
        ]
        for i in range(3):
            self.vel_body[i] += lin_accel_body[i] * dt_s
            self.rates_body[i] += ang_accel_body[i] * dt_s
            self.rpy[i] += self.rates_body[i] * dt_s
        self.rpy[0] = max(-1.2, min(1.2, self.rpy[0]))
        self.rpy[1] = max(-1.2, min(1.2, self.rpy[1]))
        self.rpy[2] = (self.rpy[2] + math.pi) % (2 * math.pi) - math.pi

        vel_ned = rotate_body_to_ned(self.rpy[0], self.rpy[1], self.rpy[2], self.vel_body)
        for i in range(3):
            self.pos_ned[i] += vel_ned[i] * dt_s
        if self.pos_ned[2] < 0.0:
            self.pos_ned[2] = 0.0
            if self.vel_body[2] < 0:
                self.vel_body[2] = 0.0

        # Specific force measured by accelerometer: a_kinematic - g_body
        g_body = rotate_ned_to_body(self.rpy[0], self.rpy[1], self.rpy[2], (0.0, 0.0, GRAVITY_MPS2))
        self.accel_body = [lin_accel_body[i] - g_body[i] for i in range(3)]

    def xsens_mtdata2_items(self):
        q = euler_to_quat_wxyz(*self.rpy)
        return (q, tuple(self.rates_body), tuple(self.accel_body))

    def depth_line(self):
        depth_m = max(0.0, self.pos_ned[2])
        press_mbar = ATM_MBAR + (WATER_DENSITY * GRAVITY_MPS2 * depth_m) / 100.0
        return f"DEPTH:{press_mbar:.2f},23.40,{depth_m:.4f}\n".encode("ascii")

    def a50_report(self, validity_us, dt_ms=100.0):
        return {
            "type": "velocity",
            "velocity_valid": True,
            "status": 0,
            "altitude": 2.0,
            "fom": 0.04,
            "time": float(dt_ms),
            "vx": round(self.vel_body[0], 5),
            "vy": round(self.vel_body[1], 5),
            "vz": round(self.vel_body[2], 5),
            "time_of_validity": int(validity_us),
        }
