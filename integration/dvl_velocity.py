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
        return DvlVelocity(velocity, vel_error, stamp, received_monotonic_s)
    except (ValueError, TypeError, KeyError, IndexError, json.JSONDecodeError):
        return None


def to_ned(sample, body_to_ned_wxyz, dvl_to_body_wxyz=(1, 0, 0, 0),
           offset_m_frd=(0, 0, 0), body_rates_rad_s=(0, 0, 0),
           now_monotonic_s=None, max_age_s=0.75):
    """Rotate body velocity to local NED and correct for the DVL lever arm.

    Attitude and body rates must come from the same fresh autopilot telemetry.
    Caller owns that freshness gate; a stale DVL sample is rejected here.
    """
    if now_monotonic_s is not None and not 0 <= now_monotonic_s - sample.received_monotonic_s <= max_age_s:
        raise ValueError("stale DVL velocity")
    vx, vy, vz = rotate(dvl_to_body_wxyz, sample.body_frd_mps)
    ox, oy, oz = _vec3(offset_m_frd)
    wx, wy, wz = _vec3(body_rates_rad_s)
    velocity_at_cg = (vx - (wy * oz - wz * oy),
                      vy - (wz * ox - wx * oz),
                      vz - (wx * oy - wy * ox))
    return rotate(body_to_ned_wxyz, velocity_at_cg)


def mavlink_covariance(error_mps):
    """ArduPilot GCS_Common.cpp consumes sqrt(trace) as velocity error."""
    if not math.isfinite(error_mps) or error_mps < 0:
        raise ValueError("invalid velocity error")
    variance = error_mps * error_mps / 3
    return (variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance)
