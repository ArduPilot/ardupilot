"""A50 bottom-track velocity to ArduPilot ExternalNav velocity measurements.

Derived from the AUV-IITK ROS 2 A50 TCP JSON reader. The A50 protocol and
frame semantics are documented at https://docs.waterlinked.com/dvl/ . This
module is pure: it never opens a socket or writes to an autopilot or actuator.
"""

from dataclasses import dataclass
import json
import math
from typing import Optional


@dataclass(frozen=True)
class DvlVelocity:
    body_frd_mps: tuple[float, float, float]
    velocity_error_mps: float
    validity_unix_us: Optional[int]
    received_monotonic_s: float
    interval_s: Optional[float] = None


@dataclass(frozen=True)
class DvlPosition:
    """A50 dead-reckoned displacement in its reset-time vehicle frame."""
    local_m: tuple[float, float, float]
    position_std_m: float
    report_time_s: float
    received_monotonic_s: float


def parse_position(line, received_monotonic_s):
    """Parse a position_local report without claiming it is navigation NED.

    The A50's origin and axes are established at startup or dead-reckoning
    reset; neither can be inferred from the JSON report alone.
    """
    try:
        report = json.loads(line)
        if not isinstance(report, dict) or report.get("type") != "position_local":
            return None
        if type(report.get("status")) is not int or report["status"] != 0:
            return None
        position = _vec3((report["x"], report["y"], report["z"]))
        std = float(report["std"])
        stamp = float(report["ts"])
        if not (math.isfinite(std) and std >= 0 and math.isfinite(stamp) and stamp >= 0):
            return None
        return DvlPosition(position, std, stamp, received_monotonic_s)
    except (ValueError, TypeError, KeyError, IndexError, json.JSONDecodeError):
        return None


def _vec3(values):
    if len(values) != 3:
        raise ValueError("expected three vector elements")
    vector = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in vector):
        raise ValueError("nonfinite vector")
    return vector


def _unit_quaternion(values):
    if len(values) != 4:
        raise ValueError("expected wxyz quaternion")
    q = tuple(float(value) for value in values)
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or not 0.9 <= norm <= 1.1:
        raise ValueError("invalid quaternion norm")
    return tuple(value / norm for value in q)


def rotate(q, v):
    """Rotate a 3-vector with a wxyz quaternion, including nonzero pitch/roll."""
    w, x, y, z = _unit_quaternion(q)
    vx, vy, vz = _vec3(v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def parse_velocity(line, received_monotonic_s, fom_max_mps=0.25):
    """Return valid bottom-track velocity, or None for nonvelocity/invalid reports.

    `time` is an inter-report interval in milliseconds, *not* a timestamp.
    `time_of_validity` is Unix microseconds in A50 JSON v3. Only the latter is
    retained for duplicate suppression; host receipt time controls freshness.
    """
    try:
        report = json.loads(line)
        if not isinstance(report, dict):
            return None
        if report.get("type", "velocity") != "velocity":
            return None
        if report.get("velocity_valid") is not True or report.get("status") != 0:
            return None
        altitude = float(report["altitude"])
        fom = float(report["fom"])
        if not (math.isfinite(altitude) and altitude >= 0.05 and
                math.isfinite(fom) and 0 <= fom <= fom_max_mps):
            return None
        velocity = _vec3((report["vx"], report["vy"], report["vz"]))
        covariance = report.get("covariance")
        if covariance is not None:
            if len(covariance) != 3 or any(len(row) != 3 for row in covariance):
                return None
            matrix = [[float(value) for value in row] for row in covariance]
            if not all(math.isfinite(value) for row in matrix for value in row):
                return None
            diagonal = [matrix[i][i] for i in range(3)]
            if not all(math.isfinite(v) and v >= 0 for v in diagonal):
                return None
            vel_error = math.sqrt(sum(diagonal))
        else:
            vel_error = fom
        if not math.isfinite(vel_error) or vel_error > fom_max_mps:
            return None
        stamp = report.get("time_of_validity")
        if stamp is not None:
            if type(stamp) is not int or stamp <= 0:
                return None
        interval_s = None
        raw_time_ms = report.get("time")
        if raw_time_ms is not None:
            val_ms = float(raw_time_ms)
            if math.isfinite(val_ms) and val_ms > 0:
                interval_s = val_ms / 1000.0
        return DvlVelocity(velocity, vel_error, stamp, received_monotonic_s, interval_s)
    except (ValueError, TypeError, KeyError, IndexError, json.JSONDecodeError):
        return None


def body_velocity_at_cg(sample, dvl_to_body_wxyz=(1, 0, 0, 0),
                        offset_m_frd=(0, 0, 0), body_rates_rad_s=(0, 0, 0),
                        now_monotonic_s=None, max_age_s=0.75):
    """Transform DVL body velocity into vehicle-body FRD velocity at CG."""
    if now_monotonic_s is not None and not 0 <= now_monotonic_s - sample.received_monotonic_s <= max_age_s:
        raise ValueError("stale DVL velocity")
    vx, vy, vz = rotate(dvl_to_body_wxyz, sample.body_frd_mps)
    ox, oy, oz = _vec3(offset_m_frd)
    wx, wy, wz = _vec3(body_rates_rad_s)
    return (vx - (wy * oz - wz * oy),
            vy - (wz * ox - wx * oz),
            vz - (wx * oy - wy * ox))


def to_ned(sample, body_to_ned_wxyz, dvl_to_body_wxyz=(1, 0, 0, 0),
           offset_m_frd=(0, 0, 0), body_rates_rad_s=(0, 0, 0),
           now_monotonic_s=None, max_age_s=0.75):
    """Rotate body velocity to local NED and correct for the DVL lever arm.

    Attitude and body rates must come from the same fresh autopilot telemetry.
    Caller owns that freshness gate; a stale DVL sample is rejected here.
    """
    velocity_at_cg = body_velocity_at_cg(
        sample, dvl_to_body_wxyz=dvl_to_body_wxyz, offset_m_frd=offset_m_frd,
        body_rates_rad_s=body_rates_rad_s, now_monotonic_s=now_monotonic_s, max_age_s=max_age_s,
    )
    return rotate(body_to_ned_wxyz, velocity_at_cg)


def to_body_delta(sample, dt_s, dvl_to_body_wxyz=(1, 0, 0, 0),
                  offset_m_frd=(0, 0, 0), body_rates_rad_s=(0, 0, 0),
                  fom_max_mps=0.25, now_monotonic_s=None, max_age_s=0.75):
    """Convert valid DVL velocity into ArduPilot VISION_POSITION_DELTA fields.

    Returns (position_delta_frd_m, angle_delta_frd_rad, time_delta_usec, confidence_pct).
    Enforces dt_s >= 0.02s so ArduPilot EKF3 bodyOdm never drops the sample.
    """
    if not math.isfinite(dt_s) or not 0.02 <= dt_s <= max_age_s:
        raise ValueError("invalid DVL integration interval")
    vel_cg = body_velocity_at_cg(
        sample, dvl_to_body_wxyz=dvl_to_body_wxyz, offset_m_frd=offset_m_frd,
        body_rates_rad_s=body_rates_rad_s, now_monotonic_s=now_monotonic_s, max_age_s=max_age_s,
    )
    wx, wy, wz = _vec3(body_rates_rad_s)
    pos_delta = (vel_cg[0] * dt_s, vel_cg[1] * dt_s, vel_cg[2] * dt_s)
    ang_delta = (wx * dt_s, wy * dt_s, wz * dt_s)
    time_delta_usec = int(round(dt_s * 1_000_000))
    scale = max(float(fom_max_mps), 1e-3)
    confidence = max(1.0, min(100.0, 100.0 * (1.0 - sample.velocity_error_mps / scale)))
    return pos_delta, ang_delta, time_delta_usec, confidence


def mavlink_covariance(error_mps):
    """ArduPilot GCS_Common.cpp consumes sqrt(trace) as velocity error."""
    if not math.isfinite(error_mps) or error_mps < 0:
        raise ValueError("invalid velocity error")
    variance = error_mps * error_mps / 3
    return (variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance)
