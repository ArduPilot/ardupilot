"""ArduSub-backed Timed Mission (RPYD sequencer) and Flight Plan runner."""

import csv
import json
import math
from pathlib import Path
import re
import threading
import time
import uuid

from timed_mission import TimedMission, validate_mission


def _wrap_deg(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


class TimedMissionRunner:
    """20 Hz RPYD target, movement, and break executor backed by ArduSubCommander."""

    def __init__(self, commander, get_snapshot, library_dir=None, clock=time.monotonic):
        self._cmd = commander
        self._get_snapshot = get_snapshot
        self._clock = clock
        self._library_dir = Path(library_dir) if library_dir else (Path(__file__).with_name("timed_missions"))
        self.engine = TimedMission()
        self.lock = threading.RLock()
        self._store_lock = threading.Lock()
        self.stopping_until = 0.0
        self.was_armed = False
        self._stop = threading.Event()
        self._thread = None

    def start_worker(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="atal-timed-mission")
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _worker(self):
        while not self._stop.wait(0.05):
            self.step()

    def fault(self):
        snap = self._get_snapshot()
        if not snap.get("link_alive"):
            return "ArduSub telemetry link unavailable"
        return ""

    def feedback(self):
        snap = self._get_snapshot()
        if not snap.get("link_alive") or not snap.get("imu_alive"):
            return None
        depth = snap.get("depth_m")
        if depth is None:
            # Fallback to pressure-derived freshwater depth only when position_alive is absent
            p = snap.get("pressure_mbar")
            if p is not None and snap.get("pressure_alive"):
                depth = max(0.0, (float(p) - 1013.25) * 100.0 / (1000.0 * 9.81))
        values = (snap.get("roll"), snap.get("pitch"), snap.get("yaw"), depth, snap.get("timestamp"))
        if any(v is None or isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
            return None
        return {
            "roll": float(values[0]),
            "pitch": float(values[1]),
            "yaw": float(values[2]),
            "depth": float(values[3]),
            "sample_time": float(values[4]),
        }

    def send(self, command):
        snap = self._get_snapshot()
        cur_depth = snap.get("depth_m")
        depth_err = (command["depth"] - cur_depth) if cur_depth is not None else 0.0
        # Normalized vertical thrust centered at 0.5 (neutral), bounded within [0.25, 0.75]
        thrust = max(0.25, min(0.75, 0.5 - 0.25 * depth_err))
        self._cmd.send_guided_attitude(
            command["roll"], command["pitch"], command["yaw"], thrust_0_1=thrust
        )
        fwd = float(command.get("forward", 0.0))
        lat = float(command.get("lateral", 0.0))
        if abs(fwd) > 1e-4 or abs(lat) > 1e-4:
            self._cmd.set_manual_control(fwd * 1000.0, lat * 1000.0, 500, 0, lease_s=0.35)

    def stop(self, reason="Operator aborted"):
        with self.lock:
            self.engine.abort(reason)
            self.stopping_until = self._clock() + 1.0
            self._disarm()

    def _disarm(self):
        try:
            self._cmd.release_manual_control()
            self._cmd.arm(False)
        except Exception:
            pass

    def step(self):
        with self.lock:
            now = self._clock()
            try:
                if self.engine.active:
                    fault = self.fault()
                    snap = self._get_snapshot()
                    armed = bool(snap.get("armed")) if snap.get("armed_known") else False
                    if self.was_armed and not armed:
                        fault = "Vehicle disarmed during mission"
                    self.was_armed = self.was_armed or armed
                    output = self.engine.tick(now, fault, None if fault else self.feedback())
                    if self.engine.active:
                        self.send(output)
                    else:
                        self.stopping_until = now + 1.0
                if now < self.stopping_until:
                    self._disarm()
            except Exception as exc:
                self.engine.abort(f"Command delivery failed: {exc}")
                self.stopping_until = now + 1.0
                self._disarm()

    def snapshot(self):
        with self.lock:
            res = self.engine.snapshot()
            res["precheck_reason"] = self.fault()
            res["measured"] = self.feedback()
            res["feedback_status"] = (
                "" if res["measured"] is not None
                else "RPYD feedback unavailable: target confirmation waits; configured deadlines remain active"
            )
            res["stopping"] = self._clock() < self.stopping_until
            return res

    def _path_for(self, mission_id):
        if not re.fullmatch(r"[0-9a-f]{32}", str(mission_id)):
            raise ValueError("Invalid mission ID")
        path = self._library_dir / f"{mission_id}.json"
        if path.is_symlink():
            raise ValueError("Invalid library path")
        return path

    def list_library(self):
        entries = []
        with self._store_lock:
            if self._library_dir.exists():
                for path in sorted(self._library_dir.glob("*.json"),
                                   key=lambda p: p.lstat().st_mtime, reverse=True):
                    if path.is_symlink() or not re.fullmatch(r"[0-9a-f]{32}", path.stem):
                        continue
                    try:
                        raw = json.loads(path.read_text(encoding="utf-8"))
                        mission = validate_mission(raw)
                        entries.append({"id": path.stem, "mission": mission})
                    except (ValueError, OSError, json.JSONDecodeError):
                        continue
        return entries

    def save_library(self, mission_data, mission_id=None):
        mission = validate_mission(mission_data)
        mid = mission_id or uuid.uuid4().hex
        with self._store_lock:
            path = self._path_for(mid)
            self._library_dir.mkdir(parents=True, exist_ok=True)
            tmp = self._library_dir / f"{mid}.{uuid.uuid4().hex}.tmp"
            tmp.write_text(json.dumps(mission, indent=2) + "\n", encoding="utf-8")
            tmp.replace(path)
        return {"ok": True, "id": mid, "mission": mission}

    def delete_library(self, mission_id):
        with self._store_lock:
            self._path_for(mission_id).unlink(missing_ok=True)
        return {"ok": True}

    def start_mission(self, mission_data):
        mission = validate_mission(mission_data)
        with self.lock:
            fault = self.fault()
            if fault or self._clock() < self.stopping_until:
                raise RuntimeError(fault or "Wait for disarm burst to finish")
            self.engine.start(mission, self._clock(), self.feedback())
            snap = self._get_snapshot()
            self.was_armed = bool(snap.get("armed")) if snap.get("armed_known") else False
            self.step()
            if not self.engine.active:
                raise RuntimeError(self.engine.reason)
        self.start_worker()
        return {"ok": True, "message": "Timed mission started"}

    def pause_mission(self):
        with self.lock:
            fault = self.fault()
            if fault:
                self.stop(fault)
                raise RuntimeError(fault)
            self.engine.pause(self._clock(), self.feedback())
            if not self.engine.active:
                self.stopping_until = self._clock() + 1.0
            self.step()
        return {"ok": True}

    def resume_mission(self):
        with self.lock:
            fault = self.fault()
            if fault:
                self.stop(fault)
                raise RuntimeError(fault)
            self.engine.resume(self._clock())
            if not self.engine.active:
                self.stopping_until = self._clock() + 1.0
            self.step()
        return {"ok": True}


class FlightPlanRunner:
    """Sequential dive/align/surge/station-hold ArduSub plan runner with evidence logs."""

    DEFAULT_SETTINGS = {
        "submerge_depth_m": 0.4,
        "dive_time_sec": 4.0,
        "yaw_angle_deg": 0.0,
        "yaw_tol_deg": 10.0,
        "yaw_timeout_sec": 12.0,
        "surge_effort": 0.25,
        "surge_time_sec": 4.0,
        "failsafe_timeout_sec": 60.0,
    }

    def __init__(self, commander, get_snapshot, log_dir=None):
        self._cmd = commander
        self._get_snapshot = get_snapshot
        self._log_dir = Path(log_dir) if log_dir else (Path(__file__).with_name("logs") / "flight_plans")
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._state = {
            "status": "idle",
            "current_step": 0,
            "total_steps": 4,
            "step_name": "Idle",
            "step_elapsed": 0.0,
            "step_duration": 0.0,
            "total_elapsed": 0.0,
            "failsafe_sec": 60.0,
            "settings": dict(self.DEFAULT_SETTINGS),
            "error": None,
            "untethered_armed": False,
            "untethered_execution": False,
            "last_log_files": None,
        }

    def status(self):
        with self._lock:
            return dict(self._state)

    def start(self, settings=None, failsafe_sec=None):
        merged = dict(self.DEFAULT_SETTINGS)
        if isinstance(settings, dict):
            for k, v in settings.items():
                if k in merged and isinstance(v, (int, float)) and math.isfinite(float(v)):
                    merged[k] = float(v)
        if failsafe_sec is not None:
            merged["failsafe_timeout_sec"] = max(5.0, min(900.0, float(failsafe_sec)))
        snap = self._get_snapshot()
        if not snap.get("link_alive"):
            return {"ok": False, "error": "No live ArduSub link"}
        with self._lock:
            if self._state["status"] == "running":
                return {"ok": False, "error": "Flight plan already running"}
            self._state.update({
                "status": "running",
                "current_step": 0,
                "step_name": "Initializing ArduSub flight plan",
                "step_elapsed": 0.0,
                "total_elapsed": 0.0,
                "failsafe_sec": merged["failsafe_timeout_sec"],
                "settings": merged,
                "error": None,
            })
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, args=(merged,), daemon=True)
        self._thread.start()
        return {"ok": True, "status": "running", "settings": merged,
                "failsafe_sec": merged["failsafe_timeout_sec"]}

    def stop(self):
        self._stop_event.set()
        self._cmd.release_manual_control()
        self._cmd.set_mode("ALT_HOLD")
        with self._lock:
            if self._state["status"] == "running":
                self._state["status"] = "aborted"
                self._state["step_name"] = "Flight plan aborted by operator"
        return {"ok": True, "status": "aborted"}

    def list_logs(self):
        if not self._log_dir.is_dir():
            return []
        sessions = []
        for path in sorted(self._log_dir.glob("flight_*.csv"),
                           key=lambda p: p.stat().st_mtime, reverse=True):
            base = path.stem
            srt = self._log_dir / f"{base}.srt"
            summary = self._log_dir / f"{base}_summary.md"
            st = path.stat()
            sessions.append({
                "session_id": base,
                "csv": path.name,
                "srt": srt.name if srt.is_file() else None,
                "summary": summary.name if summary.is_file() else None,
                "size": st.st_size,
                "mtime": st.st_mtime,
                "date": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(st.st_mtime)),
            })
        return sessions

    def resolve_log_file(self, filename):
        safe = Path(str(filename)).name
        path = self._log_dir / safe
        return path if path.is_file() else None

    def _run(self, cfg):
        self._log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = f"flight_{stamp}"
        csv_path = self._log_dir / f"{base}.csv"
        srt_path = self._log_dir / f"{base}.srt"
        summary_path = self._log_dir / f"{base}_summary.md"
        t0 = time.monotonic()
        failsafe = float(cfg["failsafe_timeout_sec"])
        milestones = []
        rows_written = 0

        steps = [
            {"idx": 1, "name": f"Dive to {cfg['submerge_depth_m']:.2f}m", "type": "dive",
             "dur": max(0.5, cfg["dive_time_sec"])},
            {"idx": 2, "name": f"Align heading to {cfg['yaw_angle_deg']:.1f}°", "type": "yaw_wait",
             "dur": max(1.0, cfg["yaw_timeout_sec"])},
            {"idx": 3, "name": f"Surge forward ({cfg['surge_effort']:.2f})", "type": "surge",
             "dur": max(0.5, cfg["surge_time_sec"])},
            {"idx": 4, "name": "Station hold", "type": "hold", "dur": 2.0},
        ]

        try:
            self._cmd.set_mode("ALT_HOLD")
            with csv_path.open("w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "elapsed_s", "step_idx", "step_name", "mode", "armed",
                    "roll_deg", "pitch_deg", "yaw_deg", "depth_m", "pressure_mbar",
                ])
                for step in steps:
                    if self._stop_event.is_set():
                        break
                    s0 = time.monotonic()
                    milestones.append((s0 - t0, step["name"]))
                    while not self._stop_event.is_set():
                        now = time.monotonic()
                        total_el = now - t0
                        step_el = now - s0
                        if total_el >= failsafe:
                            self._stop_event.set()
                            break
                        if step_el >= step["dur"]:
                            break
                        snap = self._get_snapshot()
                        with self._lock:
                            self._state.update({
                                "current_step": step["idx"],
                                "step_name": step["name"],
                                "step_elapsed": round(step_el, 1),
                                "step_duration": round(step["dur"], 1),
                                "total_elapsed": round(total_el, 1),
                            })
                        if step["type"] == "surge":
                            self._cmd.set_manual_control(cfg["surge_effort"] * 1000.0, 0, 500, 0, lease_s=0.4)
                        elif step["type"] == "yaw_wait":
                            cur_y = snap.get("yaw")
                            if cur_y is not None:
                                err = _wrap_deg(cfg["yaw_angle_deg"] - cur_y)
                                if abs(err) <= cfg["yaw_tol_deg"] and step_el >= 1.0:
                                    break
                                yaw_cmd = max(-400.0, min(400.0, err * 12.0))
                                self._cmd.set_manual_control(0, 0, 500, yaw_cmd, lease_s=0.4)
                        else:
                            self._cmd.set_manual_control(0, 0, 500, 0, lease_s=0.4)
                        writer.writerow([
                            f"{total_el:.3f}", step["idx"], step["name"],
                            snap.get("vehicle_mode") or "", snap.get("armed"),
                            snap.get("roll"), snap.get("pitch"), snap.get("yaw"),
                            snap.get("depth_m"), snap.get("pressure_mbar"),
                        ])
                        rows_written += 1
                        time.sleep(0.1)

            self._cmd.release_manual_control()
            self._cmd.arm(False)
            final_status = "aborted" if self._stop_event.is_set() else "completed"
            # Write SRT subtitle file for recorded milestones
            srt_lines = []
            for idx, (ms_t, label) in enumerate(milestones, 1):
                end_t = milestones[idx][0] if idx < len(milestones) else (time.monotonic() - t0)
                srt_lines.append(
                    f"{idx}\n{_fmt_srt(ms_t)} --> {_fmt_srt(max(ms_t + 0.5, end_t))}\n{label}\n"
                )
            srt_path.write_text("\n".join(srt_lines), encoding="utf-8")
            summary_path.write_text(
                f"# ATAL ArduSub Flight Plan Summary\n\n"
                f"- Session: `{base}`\n- Status: `{final_status}`\n"
                f"- Rows: {rows_written}\n- Duration: {time.monotonic() - t0:.1f} s\n",
                encoding="utf-8",
            )
            with self._lock:
                self._state["status"] = final_status
                self._state["step_name"] = (
                    "Flight plan complete; disarmed" if final_status == "completed"
                    else "Flight plan aborted; disarmed"
                )
                self._state["last_log_files"] = {
                    "csv": csv_path.name,
                    "srt": srt_path.name,
                    "summary": summary_path.name,
                    "row_count": rows_written,
                    "duration": round(time.monotonic() - t0, 1),
                }
        except Exception as exc:
            self._cmd.release_manual_control()
            self._cmd.arm(False)
            with self._lock:
                self._state["status"] = "error"
                self._state["error"] = str(exc)
                self._state["step_name"] = f"Error: {exc}"


def _fmt_srt(seconds):
    total_ms = max(0, int(round(seconds * 1000)))
    h = total_ms // 3_600_000
    m = (total_ms % 3_600_000) // 60_000
    s = (total_ms % 60_000) // 1000
    ms = total_ms % 1000
    return f"{h:02d}:{m:02d}:{s:02d},{ms:03d}"
