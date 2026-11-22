# AP_FLAKE8_CLEAN
"""Local CSV recorder for normalized ArduSub dashboard snapshots."""

import csv
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


FIELDS = (
    "timestamp_utc", "link_alive", "armed_known", "armed", "vehicle_mode", "imu_alive", "roll", "pitch", "yaw",
    "pressure_alive", "pressure_mbar", "water_temp", "position_alive", "depth_m", "pos_n", "pos_e",
    "vel_fwd", "vel_lat", "vel_z", "mission_alive", "mission_seq", "mission_total",
    "mission_state", "mission_mode",
    "battery_alive", "battery_v", "battery_a", "battery_percent",
    "gate_link_alive", "gate_link_ok", "gate_guidance_alive", "gate_state",
    "gate_proposal_valid", "gate_forward", "gate_lateral", "gate_vertical", "gate_yaw",
    "output_alive", *(f"pwm_{i}" for i in range(1, 9)),
)


class DashboardRecorder:
    def __init__(self, directory):
        self.directory = Path(directory)
        self._lock = threading.Lock()
        self._file = None
        self._writer = None
        self._name = None
        self._rows = 0

    def status(self):
        with self._lock:
            return {"active": self._file is not None, "file": self._name, "rows": self._rows}

    def start(self):
        with self._lock:
            if self._file is not None:
                return self._status_unlocked()
            self.directory.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            name = f"atal_telemetry_{timestamp}.csv"
            file = (self.directory / name).open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(file, fieldnames=FIELDS)
            writer.writeheader()
            file.flush()
            self._file, self._writer, self._name, self._rows = file, writer, name, 0
            return self._status_unlocked()

    def stop(self):
        with self._lock:
            if self._file is not None:
                self._file.close()
                self._file = None
                self._writer = None
            return self._status_unlocked()

    def last_file(self):
        with self._lock:
            return self.directory / self._name if self._name is not None else None

    def sample(self, snapshot, now=None):
        with self._lock:
            if self._file is None:
                return
            timestamp = datetime.fromtimestamp(time.time() if now is None else now, timezone.utc)
            row = {key: snapshot.get(key) for key in FIELDS if key != "timestamp_utc" and not key.startswith("pwm_")}
            row["timestamp_utc"] = timestamp.isoformat(timespec="milliseconds")
            if not snapshot.get("armed_known"):
                row["armed"] = None
                row["vehicle_mode"] = None
            if not snapshot.get("imu_alive"):
                for key in ("roll", "pitch", "yaw"):
                    row[key] = None
            if not snapshot.get("pressure_alive"):
                for key in ("pressure_mbar", "water_temp"):
                    row[key] = None
            if not snapshot.get("position_alive"):
                for key in ("depth_m", "pos_n", "pos_e", "vel_fwd", "vel_lat", "vel_z"):
                    row[key] = None
            elif not snapshot.get("imu_alive"):
                row["vel_fwd"] = row["vel_lat"] = None
            if not snapshot.get("mission_alive"):
                for key in ("mission_seq", "mission_total", "mission_state", "mission_mode"):
                    row[key] = None
            if not snapshot.get("battery_alive"):
                for key in ("battery_v", "battery_a", "battery_percent"):
                    row[key] = None
            if not snapshot.get("gate_link_alive"):
                row["gate_link_ok"] = None
            if not snapshot.get("gate_guidance_alive"):
                row["gate_state"] = row["gate_proposal_valid"] = None
            if not (snapshot.get("gate_link_ok") and snapshot.get("gate_guidance_alive") and snapshot.get("gate_proposal_valid")):
                for key in ("gate_forward", "gate_lateral", "gate_vertical", "gate_yaw"):
                    row[key] = None
            pwm = snapshot.get("pwm") if snapshot.get("output_alive") else None
            for index in range(8):
                row[f"pwm_{index + 1}"] = pwm[index] if isinstance(pwm, list) and len(pwm) > index else None
            self._writer.writerow(row)
            self._file.flush()
            self._rows += 1

    def _status_unlocked(self):
        return {"active": self._file is not None, "file": self._name, "rows": self._rows}
