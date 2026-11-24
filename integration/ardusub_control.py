"""Safety-gated ArduSub MAVLink command, parameter, profile, and lease manager."""

from collections import deque
import json
import math
from pathlib import Path
import threading
import time

from pymavlink import mavutil

from dashboard_telemetry import SUB_MODES


MODE_NAME_TO_ID = {name: mode_id for mode_id, name in SUB_MODES.items()}
MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
}
DEFAULT_TUNING_PARAMS = (
    "ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P",
    "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_RLL_IMAX",
    "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D", "ATC_RAT_PIT_IMAX",
    "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D", "ATC_RAT_YAW_IMAX",
    "PSC_D_POS_P", "PSC_D_VEL_P", "PSC_D_ACC_P", "PSC_D_ACC_I", "PSC_D_ACC_D",
    "PSC_NE_POS_P", "PSC_NE_VEL_P", "PSC_NE_VEL_I", "PSC_NE_VEL_D",
    "WP_SPD", "WP_SPD_DN", "WP_SPD_UP", "WP_RADIUS_M",
    "MOT_PWM_MIN", "MOT_PWM_MAX", "FRAME_CONFIG",
)


def _decode_param_id(raw):
    if isinstance(raw, bytes):
        return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return str(raw).rstrip("\x00")


def _safe_profile_name(name):
    cleaned = "".join(ch for ch in str(name).strip() if ch.isalnum() or ch in ("-", "_", "."))
    if not cleaned or cleaned in (".", ".."):
        raise ValueError("Invalid profile name")
    return cleaned


class ArduSubCommander:
    """Thread-safe MAVLink command, parameter, and manual-lease owner."""

    def __init__(self, mav_conn, send_lock=None, system_id=1, component_id=1,
                 profiles_dir=None, actuators_enabled=False,
                 allow_software_motor_test=False, allow_arming=True):
        self._conn = mav_conn
        self._send_lock = send_lock or threading.Lock()
        self._system_id = int(system_id)
        self._component_id = int(component_id)
        self._profiles_dir = Path(profiles_dir) if profiles_dir else (Path(__file__).with_name("profiles"))
        self._actuators_enabled = bool(actuators_enabled)
        self._allow_software_motor_test = bool(allow_software_motor_test)
        self._allow_arming = bool(allow_arming)

        self._lock = threading.Lock()
        self._ack_events = {}
        self._ack_results = {}
        self._params = {}
        self._param_total = None
        self._param_last_rx = 0.0
        self._param_events = {}
        self._statustexts = deque(maxlen=80)
        self._mission_handler = None

        self._manual_lock = threading.Lock()
        self._manual_active = False
        self._manual_deadline = 0.0
        self._manual_values = (0, 0, 500, 0, 0)
        self._manual_stop = threading.Event()
        self._watchdog_thread = None

    def start_watchdog(self):
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._manual_stop.clear()
        self._watchdog_thread = threading.Thread(target=self._manual_watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def close(self):
        self._manual_stop.set()
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=1.0)

    def set_mission_handler(self, handler):
        with self._lock:
            self._mission_handler = handler

    def recent_status_texts(self, limit=20):
        with self._lock:
            return list(self._statustexts)[-limit:]

    def cached_params(self):
        with self._lock:
            return {k: dict(v) for k, v in self._params.items()}

    def handle_message(self, msg):
        if hasattr(msg, "get_srcSystem") and msg.get_srcSystem() != self._system_id:
            return
        if hasattr(msg, "get_srcComponent") and msg.get_srcComponent() != self._component_id:
            return
        kind = msg.get_type()
        if kind == "COMMAND_ACK":
            cmd = int(msg.command)
            res = int(msg.result)
            with self._lock:
                self._ack_results[cmd] = {
                    "command": cmd,
                    "result": res,
                    "result_name": MAV_RESULT_NAMES.get(res, f"RESULT_{res}"),
                    "progress": getattr(msg, "progress", 0),
                    "result_param2": getattr(msg, "result_param2", 0),
                    "ts": time.monotonic(),
                }
                ev = self._ack_events.get(cmd)
                if ev is not None:
                    ev.set()
        elif kind == "PARAM_VALUE":
            name = _decode_param_id(msg.param_id)
            with self._lock:
                self._params[name] = {
                    "value": float(msg.param_value),
                    "type": int(msg.param_type),
                    "index": int(msg.param_index),
                    "count": int(msg.param_count),
                }
                self._param_total = int(msg.param_count)
                self._param_last_rx = time.monotonic()
                ev = self._param_events.get(name)
                if ev is not None:
                    ev.set()
        elif kind == "STATUSTEXT":
            text = _decode_param_id(msg.text)
            with self._lock:
                self._statustexts.append({
                    "ts": time.time(),
                    "severity": int(msg.severity),
                    "text": text,
                })
        elif kind in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK",
                      "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_ITEM"):
            with self._lock:
                handler = self._mission_handler
            if handler is not None:
                handler.handle_mavlink_message(msg)

    def send_command_long(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                          p5=0.0, p6=0.0, p7=0.0, timeout_s=2.0):
        cmd = int(command)
        ev = threading.Event()
        with self._lock:
            self._ack_results.pop(cmd, None)
            self._ack_events[cmd] = ev
        try:
            with self._send_lock:
                self._conn.mav.command_long_send(
                    self._system_id, self._component_id, cmd, 0,
                    float(p1), float(p2), float(p3), float(p4),
                    float(p5), float(p6), float(p7),
                )
            if not ev.wait(timeout=timeout_s):
                return {"ok": False, "command": cmd, "error": "COMMAND_ACK timeout",
                        "recent_status": self._latest_status_text()}
            with self._lock:
                ack = dict(self._ack_results[cmd])
            ack["ok"] = ack["result"] in (0, 5)
            ack["recent_status"] = self._latest_status_text()
            return ack
        finally:
            with self._lock:
                if self._ack_events.get(cmd) is ev:
                    self._ack_events.pop(cmd, None)

    def _latest_status_text(self):
        with self._lock:
            return self._statustexts[-1]["text"] if self._statustexts else ""

    def set_mode(self, mode_name, timeout_s=2.0):
        key = str(mode_name or "").strip().upper()
        if key not in MODE_NAME_TO_ID:
            return {"ok": False, "error": f"Unknown ArduSub mode: {mode_name}",
                    "valid_modes": sorted(MODE_NAME_TO_ID)}
        mode_id = MODE_NAME_TO_ID[key]
        res = self.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            p2=mode_id,
            timeout_s=timeout_s,
        )
        res["mode"] = key
        res["mode_id"] = mode_id
        return res

    def arm(self, armed, force=False, timeout_s=2.0):
        if armed and not self._allow_arming:
            return {"ok": False, "armed_requested": True,
                    "error": "Arming disabled by runtime safety lock"}
        p2 = 21196.0 if force else 0.0
        res = self.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1.0 if armed else 0.0,
            p2=p2,
            timeout_s=timeout_s,
        )
        res["armed_requested"] = bool(armed)
        res["forced"] = bool(force)
        return res

    def kill_all(self, timeout_s=2.0):
        """Immediately neutralize manual control and force-disarm ArduSub."""
        self.release_manual_control()
        self.clear_rc_override()
        return self.arm(False, force=True, timeout_s=timeout_s)

    def set_manual_control(self, x, y, z, r, buttons=0, lease_s=0.6):
        """Send bounded MANUAL_CONTROL with automatic deadman neutral on expiry."""
        self.start_watchdog()
        bx = max(-1000, min(1000, int(round(x))))
        by = max(-1000, min(1000, int(round(y))))
        bz = max(0, min(1000, int(round(z))))
        br = max(-1000, min(1000, int(round(r))))
        btn = int(buttons) & 0xFFFF
        with self._manual_lock:
            self._manual_active = True
            self._manual_deadline = time.monotonic() + max(0.1, float(lease_s))
            self._manual_values = (bx, by, bz, br, btn)
        self._send_manual_raw(bx, by, bz, br, btn)
        return {"ok": True, "x": bx, "y": by, "z": bz, "r": br, "lease_s": float(lease_s)}

    def release_manual_control(self):
        with self._manual_lock:
            was_active = self._manual_active
            self._manual_active = False
            self._manual_values = (0, 0, 500, 0, 0)
        if was_active:
            self._send_manual_raw(0, 0, 500, 0, 0)
        return {"ok": True}

    def clear_rc_override(self):
        with self._send_lock:
            self._conn.mav.rc_channels_override_send(
                self._system_id, self._component_id,
                0, 0, 0, 0, 0, 0, 0, 0,
            )

    def _send_manual_raw(self, x, y, z, r, buttons=0):
        with self._send_lock:
            self._conn.mav.manual_control_send(
                self._system_id, int(x), int(y), int(z), int(r), int(buttons)
            )

    def _manual_watchdog_loop(self):
        while not self._manual_stop.wait(0.05):
            send_neutral = False
            with self._manual_lock:
                if self._manual_active and time.monotonic() > self._manual_deadline:
                    self._manual_active = False
                    self._manual_values = (0, 0, 500, 0, 0)
                    send_neutral = True
            if send_neutral:
                try:
                    self._send_manual_raw(0, 0, 500, 0, 0)
                except Exception:
                    pass

    def send_guided_position_ned(self, north, east, depth_m, yaw_deg=None, yaw_rate_dps=None):
        """Command GUIDED local-NED target (depth_m is positive down)."""
        for val in (north, east, depth_m):
            if not math.isfinite(val):
                raise ValueError("Non-finite position target")
        mask = 0b0000111111111000  # x, y, z only (bit 0..2 enabled)
        yaw_rad = 0.0
        yaw_rate_rad = 0.0
        if yaw_deg is not None and math.isfinite(yaw_deg):
            mask &= ~(1 << 10)
            yaw_rad = math.radians(float(yaw_deg))
        if yaw_rate_dps is not None and math.isfinite(yaw_rate_dps):
            mask &= ~(1 << 11)
            yaw_rate_rad = math.radians(float(yaw_rate_dps))
        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                0, self._system_id, self._component_id,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mask,
                float(north), float(east), float(depth_m),
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                yaw_rad, yaw_rate_rad,
            )
        return {"ok": True, "north": float(north), "east": float(east),
                "depth_m": float(depth_m), "yaw_deg": yaw_deg}

    def send_guided_velocity_ned(self, vn, ve, vd, yaw_deg=None, yaw_rate_dps=None):
        """Command GUIDED local-NED velocity (m/s, vd positive down)."""
        for val in (vn, ve, vd):
            if not math.isfinite(val):
                raise ValueError("Non-finite velocity target")
        mask = 0b0000111111000111  # vx, vy, vz enabled (bits 3..5 = 0)
        yaw_rad = 0.0
        yaw_rate_rad = 0.0
        if yaw_deg is not None and math.isfinite(yaw_deg):
            mask &= ~(1 << 10)
            yaw_rad = math.radians(float(yaw_deg))
        if yaw_rate_dps is not None and math.isfinite(yaw_rate_dps):
            mask &= ~(1 << 11)
            yaw_rate_rad = math.radians(float(yaw_rate_dps))
        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                0, self._system_id, self._component_id,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mask,
                0.0, 0.0, 0.0,
                float(vn), float(ve), float(vd),
                0.0, 0.0, 0.0,
                yaw_rad, yaw_rate_rad,
            )
        return {"ok": True, "vn": float(vn), "ve": float(ve), "vd": float(vd)}

    def send_guided_attitude(self, roll_deg, pitch_deg, yaw_deg, thrust_0_1=0.5):
        """Command GUIDED/STABILIZE attitude quaternion + normalized throttle."""
        for val in (roll_deg, pitch_deg, yaw_deg, thrust_0_1):
            if not math.isfinite(val):
                raise ValueError("Non-finite attitude target")
        r = math.radians(float(roll_deg)) / 2.0
        p = math.radians(float(pitch_deg)) / 2.0
        y = math.radians(float(yaw_deg)) / 2.0
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        thrust = max(0.0, min(1.0, float(thrust_0_1)))
        with self._send_lock:
            self._conn.mav.set_attitude_target_send(
                0, self._system_id, self._component_id,
                0b00000111,  # ignore body rates, use attitude + thrust
                q, 0.0, 0.0, 0.0, thrust,
            )
        return {"ok": True, "roll_deg": roll_deg, "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg, "thrust": thrust}

    def set_ekf_origin(self, lat_deg, lon_deg, alt_m=0.0):
        lat_e7 = int(round(float(lat_deg) * 1e7))
        lon_e7 = int(round(float(lon_deg) * 1e7))
        alt_mm = int(round(float(alt_m) * 1000.0))
        with self._send_lock:
            self._conn.mav.set_gps_global_origin_send(
                self._system_id, lat_e7, lon_e7, alt_mm
            )
        return {"ok": True, "lat_deg": float(lat_deg), "lon_deg": float(lon_deg), "alt_m": float(alt_m)}

    def calibrate_gyro(self, timeout_s=3.0):
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, p1=1.0, timeout_s=timeout_s
        )

    def calibrate_pressure(self, timeout_s=3.0):
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, p3=1.0, timeout_s=timeout_s
        )

    def motor_test(self, motor_id, pwm_us, duration_s=1.0):
        motor = int(motor_id)
        pwm = int(pwm_us)
        dur = float(duration_s)
        if not 1 <= motor <= 8:
            return {"ok": False, "error": "motor_id must be 1..8"}
        if not 1300 <= pwm <= 1700:
            return {"ok": False, "error": "pwm_us must be within safe 1300..1700 bounds"}
        if not 0.1 <= dur <= 5.0:
            return {"ok": False, "error": "duration_s must be 0.1..5.0"}
        if not (self._actuators_enabled or self._allow_software_motor_test):
            return {"ok": False,
                    "error": "Actuators are disabled in config; motor test blocked"}
        return self.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            p1=motor,
            p2=1.0,  # MOTOR_TEST_THROTTLE_PWM
            p3=pwm,
            p4=dur,
            p5=1.0,
            timeout_s=2.0,
        )

    def refresh_params(self, timeout_s=4.0, idle_gap_s=0.35):
        with self._lock:
            self._param_last_rx = 0.0
            self._param_total = None
        with self._send_lock:
            self._conn.mav.param_request_list_send(self._system_id, self._component_id)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                count = len(self._params)
                total = self._param_total
                last_rx = self._param_last_rx
            if total and count >= total:
                break
            if last_rx > 0 and time.monotonic() - last_rx >= idle_gap_s and count > 0:
                break
            time.sleep(0.05)
        return self.cached_params()

    def get_param(self, name, timeout_s=1.5):
        key = str(name).strip()
        if not key or len(key) > 16:
            raise ValueError("ArduSub parameter names must be 1..16 chars")
        ev = threading.Event()
        with self._lock:
            self._param_events[key] = ev
        try:
            with self._send_lock:
                self._conn.mav.param_request_read_send(
                    self._system_id, self._component_id,
                    key.encode("ascii"), -1,
                )
            if not ev.wait(timeout=timeout_s):
                return None
            with self._lock:
                return dict(self._params[key])
        finally:
            with self._lock:
                if self._param_events.get(key) is ev:
                    self._param_events.pop(key, None)

    def set_param(self, name, value, timeout_s=2.0):
        key = str(name).strip()
        if not key or len(key) > 16:
            return {"ok": False, "error": "Parameter name must be 1..16 ASCII characters"}
        val = float(value)
        if not math.isfinite(val):
            return {"ok": False, "error": "Parameter value must be finite"}
        with self._lock:
            existing = self._params.get(key)
            ptype = existing["type"] if existing else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            ev = threading.Event()
            self._param_events[key] = ev
        try:
            with self._send_lock:
                self._conn.mav.param_set_send(
                    self._system_id, self._component_id,
                    key.encode("ascii"), val, ptype,
                )
            if not ev.wait(timeout=timeout_s):
                return {"ok": False, "param": key, "error": "PARAM_VALUE confirmation timeout"}
            with self._lock:
                confirmed = dict(self._params[key])
            return {"ok": True, "param": key, "value": confirmed["value"]}
        finally:
            with self._lock:
                if self._param_events.get(key) is ev:
                    self._param_events.pop(key, None)

    def list_profiles(self):
        self._profiles_dir.mkdir(parents=True, exist_ok=True)
        out = []
        for path in sorted(self._profiles_dir.glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                out.append({"name": path.stem, "saved": data.get("saved", ""),
                            "count": len(data.get("params", {}))})
            except (OSError, json.JSONDecodeError):
                continue
        return out

    def save_profile(self, name, params=None):
        safe = _safe_profile_name(name)
        self._profiles_dir.mkdir(parents=True, exist_ok=True)
        if params is None:
            cached = self.cached_params()
            params = {k: cached[k]["value"] for k in DEFAULT_TUNING_PARAMS if k in cached}
            if not params:
                params = {k: v["value"] for k, v in cached.items()}
        payload = {
            "name": safe,
            "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "params": {str(k): float(v) for k, v in params.items() if math.isfinite(float(v))},
        }
        path = self._profiles_dir / f"{safe}.json"
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return {"ok": True, "name": safe, "count": len(payload["params"])}

    def load_profile(self, name):
        safe = _safe_profile_name(name)
        path = self._profiles_dir / f"{safe}.json"
        if not path.is_file():
            return {"ok": False, "error": f"Profile {safe} not found"}
        data = json.loads(path.read_text(encoding="utf-8"))
        params = data.get("params", {})
        applied = {}
        failed = {}
        for k, v in params.items():
            res = self.set_param(k, v)
            if res.get("ok"):
                applied[k] = res["value"]
            else:
                failed[k] = res.get("error", "failed")
        return {"ok": len(failed) == 0, "name": safe, "applied": len(applied),
                "failed": failed, "params": applied}

    def delete_profile(self, name):
        safe = _safe_profile_name(name)
        path = self._profiles_dir / f"{safe}.json"
        if not path.is_file():
            return {"ok": False, "error": f"Profile {safe} not found"}
        path.unlink()
        return {"ok": True, "name": safe}
