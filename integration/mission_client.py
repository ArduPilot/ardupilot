"""ArduSub MAVLink mission upload/read-back and local-NED guided waypoint runner."""

from dataclasses import dataclass
import math
import threading
import time

from pymavlink import mavutil

from mission_queue import validate_ros_waypoints


METERS_PER_DEG_LAT = 111_319.49079327358
MISSION_TYPE_MISSION = 0


@dataclass(frozen=True)
class ArduSubMissionItem:
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int
    y: int
    z: float
    mission_type: int = MISSION_TYPE_MISSION


def local_ned_to_global(north_m, east_m, origin_lat_deg, origin_lon_deg):
    """Flat-earth projection for local pool NED offsets around an EKF origin."""
    lat_deg = float(origin_lat_deg) + float(north_m) / METERS_PER_DEG_LAT
    cos_lat = max(0.05, abs(math.cos(math.radians(float(origin_lat_deg)))))
    lon_deg = float(origin_lon_deg) + float(east_m) / (METERS_PER_DEG_LAT * cos_lat)
    return lat_deg, lon_deg


def ros_waypoints_to_mission_items(waypoints, origin_lat_deg, origin_lon_deg):
    """Convert validated ROS waypoints into ArduSub MISSION_ITEM_INT items.

    Preserves horizontal position (via global projection around EKF origin),
    positive-down depth (stored as negative relative altitude), hold_sec (param1),
    tolerance (param2), yaw (CONDITION_YAW when finite), and max_vel (DO_CHANGE_SPEED).
    """
    normalized = validate_ros_waypoints(waypoints)
    items = []
    # Item 0: home placeholder required by ArduPilot mission table.
    items.append(ArduSubMissionItem(
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        current=1,
        autocontinue=1,
        param1=0.0, param2=0.0, param3=0.0, param4=float("nan"),
        x=int(round(origin_lat_deg * 1e7)),
        y=int(round(origin_lon_deg * 1e7)),
        z=0.0,
    ))
    last_speed = None
    for wp in normalized:
        if last_speed is None or abs(wp["max_vel"] - last_speed) > 1e-4:
            items.append(ArduSubMissionItem(
                seq=len(items),
                frame=mavutil.mavlink.MAV_FRAME_MISSION,
                command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                current=0,
                autocontinue=1,
                param1=1.0,  # ground speed
                param2=float(wp["max_vel"]),
                param3=-1.0,
                param4=0.0,
                x=0, y=0, z=0.0,
            ))
            last_speed = wp["max_vel"]
        lat_deg, lon_deg = local_ned_to_global(
            wp["north"], wp["east"], origin_lat_deg, origin_lon_deg
        )
        items.append(ArduSubMissionItem(
            seq=len(items),
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0,
            autocontinue=1,
            param1=float(wp["hold_sec"]),
            param2=float(wp["tolerance"]),
            param3=0.0,
            param4=float("nan"),
            x=int(round(lat_deg * 1e7)),
            y=int(round(lon_deg * 1e7)),
            z=-float(wp["depth"]),
        ))
        items.append(ArduSubMissionItem(
            seq=len(items),
            frame=mavutil.mavlink.MAV_FRAME_MISSION,
            command=mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            current=0,
            autocontinue=1,
            param1=float(wp["yaw"]) % 360.0,
            param2=30.0,  # deg/s rate
            param3=0.0,   # shortest direction
            param4=0.0,   # absolute heading
            x=0, y=0, z=0.0,
        ))
    return items


class MissionClient:
    """MAVLink Mission Protocol upload, read-back, start, pause, abort, skip."""

    def __init__(self, commander):
        self._cmd = commander
        self._lock = threading.Lock()
        self._req_queue = []
        self._req_event = threading.Event()
        self._ack_event = threading.Event()
        self._ack_type = None
        self._count_event = threading.Event()
        self._count_value = None
        self._items_by_seq = {}
        self._item_event = threading.Event()
        self._cmd.set_mission_handler(self)

    def handle_mavlink_message(self, msg):
        kind = msg.get_type()
        with self._lock:
            if kind in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                self._req_queue.append(int(msg.seq))
                self._req_event.set()
            elif kind == "MISSION_ACK":
                self._ack_type = int(msg.type)
                self._ack_event.set()
            elif kind == "MISSION_COUNT":
                self._count_value = int(msg.count)
                self._count_event.set()
            elif kind in ("MISSION_ITEM_INT", "MISSION_ITEM"):
                self._items_by_seq[int(msg.seq)] = {
                    "seq": int(msg.seq),
                    "frame": int(msg.frame),
                    "command": int(msg.command),
                    "param1": float(msg.param1),
                    "param2": float(msg.param2),
                    "param3": float(msg.param3),
                    "param4": float(msg.param4),
                    "x": int(msg.x) if kind == "MISSION_ITEM_INT" else int(round(msg.x * 1e7)),
                    "y": int(msg.y) if kind == "MISSION_ITEM_INT" else int(round(msg.y * 1e7)),
                    "z": float(msg.z),
                }
                self._item_event.set()

    def clear_mission(self, timeout_s=3.0):
        with self._lock:
            self._ack_type = None
            self._ack_event.clear()
        with self._cmd._send_lock:
            self._cmd._conn.mav.mission_clear_all_send(
                self._cmd._system_id, self._cmd._component_id, MISSION_TYPE_MISSION
            )
        if not self._ack_event.wait(timeout=timeout_s):
            return {"ok": False, "error": "MISSION_ACK timeout on clear"}
        return {"ok": self._ack_type == 0, "ack": self._ack_type}

    def upload_items(self, items, timeout_s=6.0):
        if not items:
            return {"ok": False, "error": "Mission items cannot be empty"}
        with self._lock:
            self._req_queue.clear()
            self._req_event.clear()
            self._ack_type = None
            self._ack_event.clear()
        with self._cmd._send_lock:
            self._cmd._conn.mav.mission_count_send(
                self._cmd._system_id, self._cmd._component_id,
                len(items), MISSION_TYPE_MISSION,
            )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._ack_event.is_set():
                return {"ok": self._ack_type == 0, "ack": self._ack_type, "count": len(items)}
            self._req_event.wait(timeout=0.1)
            with self._lock:
                pending = list(self._req_queue)
                self._req_queue.clear()
                self._req_event.clear()
            for seq in pending:
                if not 0 <= seq < len(items):
                    return {"ok": False, "error": f"Autopilot requested out-of-range seq {seq}"}
                it = items[seq]
                with self._cmd._send_lock:
                    self._cmd._conn.mav.mission_item_int_send(
                        self._cmd._system_id, self._cmd._component_id,
                        it.seq, it.frame, it.command, it.current, it.autocontinue,
                        it.param1, it.param2, it.param3, it.param4,
                        it.x, it.y, it.z, it.mission_type,
                    )
        return {"ok": False, "error": "Mission upload timeout"}

    def readback(self, timeout_s=6.0):
        with self._lock:
            self._count_value = None
            self._count_event.clear()
            self._items_by_seq.clear()
        with self._cmd._send_lock:
            self._cmd._conn.mav.mission_request_list_send(
                self._cmd._system_id, self._cmd._component_id, MISSION_TYPE_MISSION
            )
        if not self._count_event.wait(timeout=min(timeout_s, 3.0)):
            return {"ok": False, "error": "MISSION_COUNT timeout"}
        count = self._count_value
        items = []
        deadline = time.monotonic() + timeout_s
        for seq in range(count):
            with self._lock:
                self._item_event.clear()
            with self._cmd._send_lock:
                self._cmd._conn.mav.mission_request_int_send(
                    self._cmd._system_id, self._cmd._component_id,
                    seq, MISSION_TYPE_MISSION,
                )
            while time.monotonic() < deadline:
                with self._lock:
                    found = self._items_by_seq.get(seq)
                if found is not None:
                    items.append(found)
                    break
                self._item_event.wait(timeout=0.1)
            else:
                return {"ok": False, "error": f"MISSION_ITEM_INT timeout at seq {seq}"}
        with self._cmd._send_lock:
            self._cmd._conn.mav.mission_ack_send(
                self._cmd._system_id, self._cmd._component_id,
                0, MISSION_TYPE_MISSION,
            )
        return {"ok": True, "count": count, "items": items}

    def start_auto_mission(self):
        return self._cmd.send_command_long(mavutil.mavlink.MAV_CMD_MISSION_START)

    def set_current(self, seq):
        with self._cmd._send_lock:
            self._cmd._conn.mav.mission_set_current_send(
                self._cmd._system_id, self._cmd._component_id, int(seq)
            )
        return {"ok": True, "seq": int(seq)}


class GuidedMissionRunner:
    """Companion-side local-NED waypoint executor backed by ArduSub GUIDED mode.

    Matches the ROS 2 Mission Builder contract (north, east, depth, yaw, max_vel,
    tolerance, hold_sec) without requiring global GPS coordinates when ArduSub
    runs in AID_RELATIVE (DVL body-odometry) or AID_ABSOLUTE.
    """

    def __init__(self, commander, get_snapshot):
        self._cmd = commander
        self._get_snapshot = get_snapshot
        self._lock = threading.Lock()
        self._wps = []
        self._status = "idle"
        self._current_idx = -1
        self._hold_start = 0.0
        self._elapsed_hold = 0.0
        self._dist_to_wp = None
        self._last_error = ""
        self._stop = threading.Event()
        self._thread = None

    def set_queue(self, wps):
        normalized = validate_ros_waypoints(wps)
        with self._lock:
            self._wps = normalized
            if self._status in ("completed", "aborted"):
                self._status = "idle"
                self._current_idx = -1
        return {"ok": True, "count": len(normalized), "wps": normalized}

    def start(self, wps=None):
        if wps is not None:
            self.set_queue(wps)
        with self._lock:
            if not self._wps:
                return {"ok": False, "error": "Mission queue is empty"}
            if self._current_idx < 0 or self._current_idx >= len(self._wps):
                self._current_idx = 0
            idx = self._current_idx
            wp = self._wps[idx]
        snap = self._get_snapshot()
        if not snap.get("link_alive"):
            return {"ok": False, "error": "No live ArduSub telemetry link"}
        mode_res = self._cmd.set_mode("GUIDED")
        if not mode_res.get("ok"):
            return {"ok": False, "error": mode_res.get("recent_status") or "Failed to enter GUIDED mode",
                    "mode_ack": mode_res}
        self._cmd.set_param("WP_SPD", max(0.1, min(20.0, float(wp["max_vel"]))))
        self._cmd.send_guided_position_ned(wp["north"], wp["east"], wp["depth"], wp["yaw"])
        with self._lock:
            self._status = "running"
            self._hold_start = 0.0
            self._elapsed_hold = 0.0
            self._last_error = ""
        self._ensure_thread()
        return {"ok": True, "status": "running", "current_idx": idx}

    def pause(self):
        with self._lock:
            self._status = "paused"
            self._hold_start = 0.0
        snap = self._get_snapshot()
        if snap.get("position_alive") and snap.get("pos_n") is not None:
            self._cmd.send_guided_position_ned(
                snap["pos_n"], snap["pos_e"], snap.get("depth_m") or 0.0, snap.get("yaw")
            )
        return {"ok": True, "status": "paused"}

    def abort(self):
        with self._lock:
            self._status = "idle"
            self._current_idx = -1
            self._hold_start = 0.0
            self._elapsed_hold = 0.0
            self._dist_to_wp = None
        self._cmd.set_mode("ALT_HOLD")
        return {"ok": True, "status": "idle"}

    def skip(self):
        with self._lock:
            if not self._wps:
                return {"ok": False, "error": "Mission queue is empty"}
            next_idx = self._current_idx + 1
            if next_idx >= len(self._wps):
                self._status = "completed"
                self._current_idx = len(self._wps)
                self._hold_start = 0.0
                return {"ok": True, "completed": True, "status": "completed"}
            self._current_idx = next_idx
            self._hold_start = 0.0
            self._elapsed_hold = 0.0
            wp = self._wps[next_idx]
            running = self._status == "running"
        if running:
            self._cmd.set_param("WP_SPD", max(0.1, min(20.0, float(wp["max_vel"]))))
            self._cmd.send_guided_position_ned(wp["north"], wp["east"], wp["depth"], wp["yaw"])
        return {"ok": True, "current_idx": next_idx}

    def status(self):
        with self._lock:
            hold_dur = (self._wps[self._current_idx]["hold_sec"]
                        if 0 <= self._current_idx < len(self._wps) else 0.0)
            return {
                "status": self._status,
                "current_idx": self._current_idx,
                "total_wps": len(self._wps),
                "dist_to_wp": self._dist_to_wp,
                "hold_duration": hold_dur,
                "elapsed_hold": self._elapsed_hold,
                "last_error": self._last_error,
                "wps": [dict(w) for w in self._wps],
            }

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _ensure_thread(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(0.1):
            with self._lock:
                if self._status != "running":
                    continue
                if not 0 <= self._current_idx < len(self._wps):
                    self._status = "completed"
                    continue
                wp = self._wps[self._current_idx]
            snap = self._get_snapshot()
            if not snap.get("link_alive") or not snap.get("position_alive"):
                with self._lock:
                    self._last_error = "Position or link telemetry unavailable"
                continue
            dn = wp["north"] - snap["pos_n"]
            de = wp["east"] - snap["pos_e"]
            dist = math.hypot(dn, de)
            now = time.monotonic()
            advance_wp = None
            with self._lock:
                self._dist_to_wp = dist
                if dist <= wp["tolerance"]:
                    if self._hold_start == 0.0:
                        self._hold_start = now
                    self._elapsed_hold = now - self._hold_start
                    if self._elapsed_hold >= wp["hold_sec"]:
                        next_idx = self._current_idx + 1
                        if next_idx >= len(self._wps):
                            self._status = "completed"
                            self._current_idx = len(self._wps)
                            self._hold_start = 0.0
                        else:
                            self._current_idx = next_idx
                            self._hold_start = 0.0
                            self._elapsed_hold = 0.0
                            advance_wp = self._wps[next_idx]
                else:
                    self._hold_start = 0.0
                    self._elapsed_hold = 0.0
            if advance_wp is not None:
                self._cmd.set_param("WP_SPD", max(0.1, min(20.0, float(advance_wp["max_vel"]))))
                self._cmd.send_guided_position_ned(
                    advance_wp["north"], advance_wp["east"],
                    advance_wp["depth"], advance_wp["yaw"],
                )
