"""Read-only MAVLink to legacy ATAL dashboard-state mapping.

No command, arm, parameter, serial, or motor interface lives in this module.
"""

import math
import time


class DashboardTelemetry:
    def __init__(self, now=time.monotonic, system_id=1):
        self._now = now
        self._system_id = system_id
        self._heartbeat_at = None
        self._attitude_at = None
        self._pressure_at = None
        self._position_at = None
        self._servo_at = None
        self._state = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "gyro": [0.0, 0.0, 0.0], "pwm": [1500] * 8,
            "armed": False, "armed_known": False,
            "pressure_mbar": None, "water_temp": None, "depth_m": None,
            "pos_n": None, "pos_e": None, "vel_fwd": None,
            "vel_lat": None, "vel_z": None, "dvl_locked": None,
            "timestamp": 0.0, "imu_hz": 0.0,
        }

    def ingest(self, msg):
        if hasattr(msg, "get_srcSystem") and msg.get_srcSystem() != self._system_id:
            return
        if hasattr(msg, "get_srcComponent") and msg.get_srcComponent() != 1:
            return
        kind = msg.get_type()
        now = self._now()
        s = self._state
        if kind == "HEARTBEAT":
            self._heartbeat_at = now
            s["armed"] = bool(msg.base_mode & 128)
        elif kind == "ATTITUDE":
            values = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
            if not all(math.isfinite(v) for v in values):
                return
            s["roll"], s["pitch"], s["yaw"] = (math.degrees(v) for v in values[:3])
            s["gyro"] = list(values[3:])
            if self._attitude_at is not None and now > self._attitude_at:
                s["imu_hz"] = 1.0 / (now - self._attitude_at)
            self._attitude_at = now
            s["timestamp"] = time.time()
        elif kind == "SCALED_PRESSURE":
            if math.isfinite(msg.press_abs) and math.isfinite(msg.temperature):
                s["pressure_mbar"] = float(msg.press_abs)
                s["water_temp"] = float(msg.temperature) / 100.0
                self._pressure_at = now
        elif kind == "LOCAL_POSITION_NED":
            values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
            if not all(math.isfinite(v) for v in values):
                return
            s["pos_n"], s["pos_e"] = float(msg.x), float(msg.y)
            s["depth_m"] = float(msg.z)  # NED z is positive downward
            self._position_at = now
            if self._attitude_at is not None and now - self._attitude_at < 2.5:
                yaw = math.radians(s["yaw"])
                s["vel_fwd"] = math.cos(yaw) * msg.vx + math.sin(yaw) * msg.vy
                s["vel_lat"] = -math.sin(yaw) * msg.vx + math.cos(yaw) * msg.vy
            else:
                s["vel_fwd"] = s["vel_lat"] = None
            s["vel_z"] = float(msg.vz)
        elif kind == "SERVO_OUTPUT_RAW" and msg.port == 0:
            values = [getattr(msg, f"servo{i}_raw") for i in range(1, 9)]
            if all(900 <= v <= 2100 for v in values):
                s["pwm"] = values
                self._servo_at = now

    def snapshot(self):
        now = self._now()
        s = self._state.copy()
        s["gyro"] = self._state["gyro"][:]
        s["pwm"] = self._state["pwm"][:]
        s["link_alive"] = self._heartbeat_at is not None and now - self._heartbeat_at < 3.0
        s["armed_known"] = s["link_alive"]
        if not s["link_alive"]:
            s["armed"] = None
        s["imu_age"] = now - self._attitude_at if self._attitude_at is not None else None
        s["imu_alive"] = s["imu_age"] is not None and s["imu_age"] < 2.5
        if not s["imu_alive"]:
            for key in ("roll", "pitch", "yaw", "gyro", "imu_hz"):
                s[key] = None
        s["pressure_alive"] = self._pressure_at is not None and now - self._pressure_at < 2.5
        if not s["pressure_alive"]:
            s["pressure_mbar"] = s["water_temp"] = None
        s["position_alive"] = self._position_at is not None and now - self._position_at < 2.5
        if not s["position_alive"]:
            for key in ("pos_n", "pos_e", "depth_m", "vel_fwd", "vel_lat", "vel_z"):
                s[key] = None
        elif not s["imu_alive"]:
            s["vel_fwd"] = s["vel_lat"] = None
        s["output_alive"] = self._servo_at is not None and now - self._servo_at < 2.5
        if not s["output_alive"]:
            s["pwm"] = [None] * 8
        s["source"] = "ardusub_mavlink"
        return s
