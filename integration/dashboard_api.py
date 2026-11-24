"""Local ArduSub dashboard: telemetry, recording, and optional ArduSub control API.

Read-only by default (`python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650`).
Pass `--enable-control` to activate the ArduSub MAVLink command, mission,
parameter, flight-plan, and diagnostics routes for `/classic`.
"""

import argparse
import base64
import json
from pathlib import Path
import socket
import subprocess
import threading
import time

from flask import Flask, Response, jsonify, request, send_file, send_from_directory
from pymavlink import mavutil

from ardusub_control import ArduSubCommander, DEFAULT_TUNING_PARAMS
from atal_runtime import RuntimeSupervisor, list_serial_ports
from classic_dashboard import render_classic_dashboard
from dashboard_recorder import DashboardRecorder
from dashboard_telemetry import DashboardTelemetry, SUB_MODES
from flight_plan_runner import FlightPlanRunner, TimedMissionRunner
from gate_bridge import GateGuidanceBridge
from mission_client import (
    GuidedMissionRunner,
    MissionClient,
    ros_waypoints_to_mission_items,
)
from mission_queue import validate_ros_waypoints


DEFAULT_ORIGIN_LAT = 26.512339  # Provisional reference origin; override via EKF origin
DEFAULT_ORIGIN_LON = 80.232900


def make_app(telemetry, lock, recorder=None, demo_mode=False, commander=None,
             mission_client=None, guided_mission=None, timed_mission=None,
             flight_plan=None, runtime=None, gate_bridge=None, config_path=None):
    app = Flask(__name__, template_folder="reference_dashboard/templates")
    cfg_path = Path(config_path) if config_path else (Path(__file__).with_name("vehicle_defaults.json"))
    control_enabled = commander is not None

    @app.get("/classic")
    def classic_dashboard():
        response = Response(
            render_classic_dashboard(control_enabled=control_enabled),
            mimetype="text/html",
        )
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; script-src 'self'; connect-src 'self'; "
            "style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; "
            "font-src 'self' https://fonts.gstatic.com; img-src 'self' data:"
        )
        response.headers["X-Content-Type-Options"] = "nosniff"
        return response

    @app.get("/reference-static/<path:filename>")
    def reference_static(filename):
        return send_from_directory(Path(__file__).with_name("reference_dashboard") / "static", filename)

    @app.get("/")
    def dashboard():
        return send_file(Path(__file__).with_name("operator_dashboard.html"))

    @app.get("/diagnostics")
    def diagnostics():
        return send_file(Path(__file__).with_name("dashboard_view.html"))

    @app.get("/debug")
    def debug_page():
        return send_file(Path(__file__).with_name("reference_dashboard") / "templates" / "debug.html")

    @app.get("/api/state")
    def state():
        with lock:
            snapshot = telemetry.snapshot()
        snapshot["demo_mode"] = demo_mode
        snapshot["control_enabled"] = control_enabled
        if gate_bridge is not None:
            snapshot["gate_bridge_enabled"] = gate_bridge.enabled
        if recorder is not None:
            snapshot["logger_active"] = bool(recorder.status().get("active"))
        return jsonify(snapshot)

    @app.get("/api/health")
    def health():
        with lock:
            snapshot = telemetry.snapshot()
        return jsonify({key: snapshot[key] for key in (
            "link_alive", "imu_alive", "pressure_alive", "position_alive", "ekf_status_alive",
            "ekf_attitude_ok", "ekf_velocity_horizontal_ok", "ekf_relative_position_ok",
            "ekf_absolute_position_ok", "mission_alive", "battery_alive", "dvl_alive",
            "gate_link_alive", "gate_guidance_alive", "gate_link_ok", "armed_known", "source"
        )})

    @app.get("/api/stream")
    def stream():
        # Observing state never triggers vehicle commands or deadman side effects.
        def events():
            while True:
                with lock:
                    snapshot = telemetry.snapshot()
                snapshot["demo_mode"] = demo_mode
                snapshot["control_enabled"] = control_enabled
                if gate_bridge is not None:
                    snapshot["gate_bridge_enabled"] = gate_bridge.enabled
                if recorder is not None:
                    snapshot["logger_active"] = bool(recorder.status().get("active"))
                yield f"data: {json.dumps(snapshot)}\n\n"
                time.sleep(0.1)

        response = Response(events(), mimetype="text/event-stream")
        response.headers["Cache-Control"] = "no-cache, no-transform"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    @app.get("/api/recording")
    def recording_status():
        return jsonify(recorder.status() if recorder is not None else {"active": False, "file": None, "rows": 0})

    @app.post("/api/mission/validate")
    def mission_validate():
        body = request.get_json(silent=True)
        if not isinstance(body, dict):
            return jsonify({"error": "Expected a JSON object with wps"}), 400
        try:
            waypoints = validate_ros_waypoints(body.get("wps"))
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400
        return jsonify({"count": len(waypoints), "frame": "local_ned_unaligned",
                        "upload_ready": control_enabled, "wps": waypoints})

    @app.post("/api/recording/start")
    def recording_start():
        if recorder is None:
            return jsonify({"error": "Recording unavailable"}), 503
        if request.headers.get("X-ATAL-Local-Action") != "recording":
            return jsonify({"error": "Local recording action header required"}), 403
        return jsonify(recorder.start())

    @app.post("/api/recording/stop")
    def recording_stop():
        if recorder is None:
            return jsonify({"error": "Recording unavailable"}), 503
        if request.headers.get("X-ATAL-Local-Action") != "recording":
            return jsonify({"error": "Local recording action header required"}), 403
        return jsonify(recorder.stop())

    @app.get("/api/recording/download")
    def recording_download():
        path = recorder.last_file() if recorder is not None else None
        if path is None or not path.is_file():
            return jsonify({"error": "No recording available"}), 404
        return send_file(path, mimetype="text/csv", as_attachment=True, download_name=path.name)

    if commander is None:
        return app

    # ── Interactive ArduSub control routes (enabled only when commander is attached) ──

    def _snapshot():
        with lock:
            return telemetry.snapshot()

    def _actuators_enabled():
        try:
            return bool(json.loads(cfg_path.read_text(encoding="utf-8"))["actuators"].get("enabled"))
        except Exception:
            return False

    def _abort_active_sequencers():
        if timed_mission is not None and timed_mission.engine.active:
            timed_mission.stop("Emergency/manual override")
        if flight_plan is not None and flight_plan.status().get("status") == "running":
            flight_plan.stop()
        if guided_mission is not None and guided_mission.status().get("status") == "running":
            guided_mission.abort()
        if gate_bridge is not None:
            gate_bridge.set_enabled(False)

    @app.post("/api/arm")
    def api_arm():
        body = request.get_json(silent=True) or {}
        if "arm" in body:
            target = bool(body["arm"])
        else:
            cur = _snapshot().get("armed")
            target = not bool(cur)
        force = bool(body.get("force", False))
        if not target:
            _abort_active_sequencers()
        res = commander.arm(target, force=force)
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/kill")
    @app.post("/api/kill_all")
    def api_kill_all():
        _abort_active_sequencers()
        res = commander.kill_all()
        return jsonify(res), (200 if res.get("ok") else 502)

    @app.post("/api/set_mode")
    def api_set_mode():
        body = request.get_json(silent=True) or {}
        mode = body.get("mode", "")
        res = commander.set_mode(mode)
        return jsonify(res), (200 if res.get("ok") else 400)

    @app.post("/api/set_target")
    def api_set_target():
        body = request.get_json(silent=True) or {}
        mode = str(body.get("mode", "setpoint")).lower()
        if mode == "hold":
            commander.release_manual_control()
            snap = _snapshot()
            pos_ok = snap.get("ekf_relative_position_ok") or snap.get("ekf_absolute_position_ok")
            target_mode = "POSHOLD" if pos_ok else "ALT_HOLD"
            res = commander.set_mode(target_mode)
            return jsonify(res), (200 if res.get("ok") else 409)
        if mode == "manual" or ("x" in body and "y" in body and "z" in body and "r" in body):
            res = commander.set_manual_control(
                body.get("x", 0), body.get("y", 0), body.get("z", 500), body.get("r", 0),
                lease_s=float(body.get("lease_s", 0.6)),
            )
            return jsonify(res)
        if "north" in body and "east" in body:
            depth = float(body.get("depth", 0.0))
            yaw = body.get("yaw")
            res = commander.send_guided_position_ned(float(body["north"]), float(body["east"]), depth, yaw)
            return jsonify(res)
        if "fwd" in body or "lat" in body or "vz" in body:
            fwd = float(body.get("fwd", 0.0))
            lat = float(body.get("lat", 0.0))
            yaw = float(body.get("yaw", _snapshot().get("yaw") or 0.0))
            vz = float(body.get("vz", 0.0))
            if abs(fwd) > 1e-4 or abs(lat) > 1e-4 or abs(vz) > 1e-4:
                res = commander.set_manual_control(
                    fwd * 1000.0, lat * 1000.0, 500.0 - vz * 400.0, 0.0, lease_s=0.6
                )
                return jsonify(res)
        roll = float(body.get("roll", 0.0))
        pitch = float(body.get("pitch", 0.0))
        yaw = float(body.get("yaw", _snapshot().get("yaw") or 0.0))
        depth = float(body.get("depth", _snapshot().get("depth_m") or 0.0))
        cur_depth = _snapshot().get("depth_m")
        thrust = 0.5 if cur_depth is None else max(0.25, min(0.75, 0.5 - 0.25 * (depth - cur_depth)))
        res = commander.send_guided_attitude(roll, pitch, yaw, thrust_0_1=thrust)
        return jsonify(res)

    @app.post("/api/set_target_depth")
    def api_set_target_depth():
        body = request.get_json(silent=True) or {}
        depth = float(body.get("depth", 0.0))
        snap = _snapshot()
        n = snap.get("pos_n") or 0.0
        e = snap.get("pos_e") or 0.0
        if snap.get("vehicle_mode") == "GUIDED":
            res = commander.send_guided_position_ned(n, e, depth, snap.get("yaw"))
        else:
            res = commander.set_mode("ALT_HOLD")
        res["depth"] = depth
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/set_target_waypoint")
    def api_set_target_waypoint():
        body = request.get_json(silent=True) or {}
        north = float(body.get("north", 0.0))
        east = float(body.get("east", 0.0))
        depth = float(body.get("depth", 0.0))
        yaw = body.get("yaw")
        mode_res = commander.set_mode("GUIDED")
        if not mode_res.get("ok"):
            return jsonify(mode_res), 409
        return jsonify(commander.send_guided_position_ned(north, east, depth, yaw))

    @app.post("/api/hold_position")
    def api_hold_position():
        commander.release_manual_control()
        snap = _snapshot()
        if snap.get("vehicle_mode") == "GUIDED" and snap.get("position_alive") and snap.get("pos_n") is not None:
            res = commander.send_guided_position_ned(
                snap["pos_n"], snap["pos_e"], snap.get("depth_m") or 0.0, snap.get("yaw")
            )
            return jsonify(res)
        target_mode = "POSHOLD" if snap.get("ekf_relative_position_ok") or snap.get("ekf_absolute_position_ok") else "ALT_HOLD"
        res = commander.set_mode(target_mode)
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/reset_gyro")
    def api_reset_gyro():
        res = commander.calibrate_gyro()
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/tare_depth")
    def api_tare_depth():
        res = commander.calibrate_pressure()
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/reset_dvl_origin")
    def api_reset_dvl_origin():
        body = request.get_json(silent=True) or {}
        lat = float(body.get("lat", DEFAULT_ORIGIN_LAT))
        lon = float(body.get("lon", DEFAULT_ORIGIN_LON))
        alt = float(body.get("alt", 0.0))
        return jsonify(commander.set_ekf_origin(lat, lon, alt))

    @app.post("/api/reset_pids")
    def api_reset_pids():
        commander.release_manual_control()
        return jsonify({"ok": True, "message": "Manual control zeroed; ArduSub controller state active"})

    @app.get("/api/get_params")
    def api_get_params():
        params = commander.cached_params()
        if not params or request.args.get("refresh") == "1":
            params = commander.refresh_params(timeout_s=3.5)
        flat = {k: v["value"] for k, v in sorted(params.items())}
        return jsonify({"ok": True, "count": len(flat), "params": flat,
                        "tuning_keys": [k for k in DEFAULT_TUNING_PARAMS if k in flat]})

    @app.post("/api/set_param")
    def api_set_param():
        body = request.get_json(silent=True) or {}
        param = body.get("param")
        if param is None or "value" not in body:
            return jsonify({"ok": False, "error": "param and value required"}), 400
        res = commander.set_param(param, body["value"])
        return jsonify(res), (200 if res.get("ok") else 400)

    @app.post("/api/set_params_batch")
    @app.post("/api/save_params")
    def api_save_params():
        body = request.get_json(silent=True) or {}
        params = body.get("params") or {}
        applied = {}
        failed = {}
        for k, v in params.items():
            res = commander.set_param(k, v)
            if res.get("ok"):
                applied[k] = res["value"]
            else:
                failed[k] = res.get("error", "failed")
        return jsonify({"ok": len(failed) == 0, "applied": applied, "failed": failed})

    @app.get("/api/profiles")
    def api_profiles():
        return jsonify({"ok": True, "profiles": commander.list_profiles()})

    @app.post("/api/save_profile")
    def api_save_profile():
        body = request.get_json(silent=True) or {}
        try:
            res = commander.save_profile(body.get("name", ""), body.get("params"))
            return jsonify(res)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/load_profile")
    def api_load_profile():
        body = request.get_json(silent=True) or {}
        try:
            res = commander.load_profile(body.get("name", ""))
            return jsonify(res), (200 if res.get("ok") else 404)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/delete_profile")
    def api_delete_profile():
        body = request.get_json(silent=True) or {}
        try:
            res = commander.delete_profile(body.get("name", ""))
            return jsonify(res), (200 if res.get("ok") else 404)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    # ── Mission Builder (Guided local NED + MAVLink Mission Protocol upload/readback) ──

    @app.post("/api/mission/set_queue")
    def api_mission_set_queue():
        body = request.get_json(silent=True) or {}
        try:
            return jsonify(guided_mission.set_queue(body.get("wps", [])))
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/mission/start")
    def api_mission_start():
        body = request.get_json(silent=True) or {}
        try:
            res = guided_mission.start(body.get("wps"))
            return jsonify(res), (200 if res.get("ok") else 409)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/mission/pause")
    def api_mission_pause():
        return jsonify(guided_mission.pause())

    @app.post("/api/mission/abort")
    def api_mission_abort():
        return jsonify(guided_mission.abort())

    @app.post("/api/mission/skip")
    def api_mission_skip():
        res = guided_mission.skip()
        return jsonify(res), (200 if res.get("ok") else 400)

    @app.get("/api/mission/status")
    def api_mission_status():
        return jsonify(guided_mission.status())

    @app.post("/api/mission/upload")
    def api_mission_upload():
        if mission_client is None:
            return jsonify({"ok": False, "error": "Mission protocol unavailable"}), 503
        body = request.get_json(silent=True) or {}
        origin_lat = float(body.get("origin_lat", DEFAULT_ORIGIN_LAT))
        origin_lon = float(body.get("origin_lon", DEFAULT_ORIGIN_LON))
        try:
            items = ros_waypoints_to_mission_items(body.get("wps", []), origin_lat, origin_lon)
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        commander.set_ekf_origin(origin_lat, origin_lon, 0.0)
        up = mission_client.upload_items(items)
        if not up.get("ok"):
            return jsonify(up), 502
        rb = mission_client.readback()
        return jsonify({"ok": rb.get("ok", False), "uploaded": len(items),
                        "readback_count": rb.get("count"), "items": rb.get("items", [])})

    @app.get("/api/mission/readback")
    def api_mission_readback():
        if mission_client is None:
            return jsonify({"ok": False, "error": "Mission protocol unavailable"}), 503
        res = mission_client.readback()
        return jsonify(res), (200 if res.get("ok") else 502)

    # ── Timed Missions (20 Hz RPYD/move/break) ──

    @app.get("/api/timed-missions/library")
    def api_tm_library():
        return jsonify({"ok": True, "missions": timed_mission.list_library()})

    @app.post("/api/timed-missions/library")
    def api_tm_save():
        body = request.get_json(silent=True) or {}
        try:
            return jsonify(timed_mission.save_library(body.get("mission"), body.get("id")))
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.delete("/api/timed-missions/library/<mission_id>")
    def api_tm_delete(mission_id):
        try:
            return jsonify(timed_mission.delete_library(mission_id))
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.get("/api/timed-missions/state")
    def api_tm_state():
        return jsonify({"ok": True, **timed_mission.snapshot()})

    @app.post("/api/timed-missions/start")
    def api_tm_start():
        body = request.get_json(silent=True) or {}
        try:
            return jsonify(timed_mission.start_mission(body.get("mission")))
        except (ValueError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.post("/api/timed-missions/pause")
    def api_tm_pause():
        try:
            return jsonify(timed_mission.pause_mission())
        except (ValueError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 409

    @app.post("/api/timed-missions/resume")
    def api_tm_resume():
        try:
            return jsonify(timed_mission.resume_mission())
        except (ValueError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 409

    @app.post("/api/timed-missions/abort")
    def api_tm_abort():
        timed_mission.stop("Operator aborted")
        return jsonify({"ok": True, "message": "Timed mission aborted and disarm requested"})

    # ── Flight Plan ──

    @app.post("/api/flight_plan/start")
    def api_fp_start():
        body = request.get_json(silent=True) or {}
        res = flight_plan.start(body.get("settings"), body.get("failsafe_sec"))
        return jsonify(res), (200 if res.get("ok") else 409)

    @app.post("/api/flight_plan/stop")
    def api_fp_stop():
        return jsonify(flight_plan.stop())

    @app.get("/api/flight_plan/status")
    def api_fp_status():
        return jsonify(flight_plan.status())

    @app.get("/api/flight_plan/logs")
    def api_fp_logs():
        return jsonify({"sessions": flight_plan.list_logs()})

    @app.get("/api/flight_plan/download/<path:filename>")
    def api_fp_download(filename):
        path = flight_plan.resolve_log_file(filename)
        if path is None:
            return jsonify({"error": "File not found"}), 404
        return send_file(path, as_attachment=True, download_name=path.name)

    @app.post("/api/flight_plan/arm_untethered")
    def api_fp_arm_untethered():
        return jsonify({
            "ok": False,
            "error": "Untethered tether-drop launch is disabled until hardware E-stop & comms failsafe qualification",
        }), 403

    @app.post("/api/flight_plan/cancel_untethered")
    def api_fp_cancel_untethered():
        return jsonify({"ok": True, "status": "idle"})

    @app.post("/api/flight_plan/resync_srt")
    def api_fp_resync_srt():
        return jsonify({"ok": False, "error": "Camera clock alignment not yet configured"}), 501

    # ── Motor layout & software-gated motor tests ──

    @app.get("/api/motor_pins")
    def api_motor_pins():
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
        pins = {str(m["logical_id"]): m["esp32_gpio"] for m in cfg["actuators"]["motors"]}
        return jsonify({"ok": True, "pins": pins, "actuators_enabled": _actuators_enabled()})

    @app.post("/api/update_motor_pin")
    def api_update_motor_pin():
        return jsonify({"ok": False, "error": "Edit integration/vehicle_defaults.json and rerun check_layout.py"}), 403

    @app.post("/api/motor_test_pwm")
    def api_motor_test_pwm():
        body = request.get_json(silent=True) or {}
        res = commander.motor_test(
            body.get("motor", 0), body.get("pwm", 1500), body.get("duration_s", 1.0)
        )
        return jsonify(res), (200 if res.get("ok") else 403)

    @app.post("/api/motor_test_stop")
    @app.post("/api/motor_neutral")
    def api_motor_neutral():
        commander.release_manual_control()
        return jsonify({"ok": True, "message": "Motor test neutral"})

    @app.post("/api/set_motor_direction")
    def api_set_motor_direction():
        body = request.get_json(silent=True) or {}
        motor = int(body.get("motor", 0))
        direction = int(body.get("direction", 1))
        if not 1 <= motor <= 8 or direction not in (-1, 1):
            return jsonify({"ok": False, "error": "motor 1..8 and direction -1/1 required"}), 400
        return jsonify(commander.set_param(f"MOT_{motor}_DIRECTION", direction))

    @app.post("/api/set_motor_enabled")
    @app.post("/api/set_motor_weight")
    @app.post("/api/set_axis_enable")
    def api_axis_unsupported():
        return jsonify({
            "ok": False,
            "error": "ArduSub manages per-axis control through flight modes (STABILIZE/ALT_HOLD/POSHOLD/GUIDED)",
        }), 501

    # ── Logging, screenshots, runtime, ports, debug, gate bridge, camera ──

    @app.post("/api/log/start")
    def api_log_start():
        if recorder is None:
            return jsonify({"ok": False, "error": "Recorder unavailable"}), 503
        return jsonify({"ok": True, **recorder.start()})

    @app.post("/api/log/stop")
    def api_log_stop():
        if recorder is None:
            return jsonify({"ok": False, "error": "Recorder unavailable"}), 503
        return jsonify({"ok": True, **recorder.stop()})

    @app.get("/api/log/status")
    def api_log_status():
        return jsonify(recorder.status() if recorder is not None else {"active": False})

    @app.get("/api/log/list")
    def api_log_list():
        default_dir = Path("integration/logs/dashboard")
        log_dir = Path(recorder.status().get("dir", str(default_dir))) if recorder else default_dir
        files = [p.name for p in sorted(log_dir.glob("*.csv"), reverse=True)] if log_dir.is_dir() else []
        return jsonify({"files": files})

    @app.get("/api/log/download/<path:filename>")
    def api_log_download(filename):
        default_dir = Path("integration/logs/dashboard")
        log_dir = Path(recorder.status().get("dir", str(default_dir))) if recorder else default_dir
        target = log_dir / Path(filename).name
        if not target.is_file():
            return jsonify({"error": "Log not found"}), 404
        return send_file(target, mimetype="text/csv", as_attachment=True, download_name=target.name)

    @app.post("/api/save_screenshot")
    def api_save_screenshot():
        body = request.get_json(silent=True) or {}
        image = body.get("image", "")
        if not isinstance(image, str) or "," not in image:
            return jsonify({"ok": False, "error": "Invalid image data"}), 400
        _, encoded = image.split(",", 1)
        try:
            raw = base64.b64decode(encoded, validate=True)
        except Exception:
            return jsonify({"ok": False, "error": "Invalid base64 image"}), 400
        shots = Path(__file__).with_name("logs") / "screenshots"
        shots.mkdir(parents=True, exist_ok=True)
        page = "".join(c for c in str(body.get("page", "dashboard")) if c.isalnum() or c in "-_").lower() or "dashboard"
        name = f"screenshot_{page}_{time.strftime('%Y%m%d_%H%M%S')}.png"
        (shots / name).write_bytes(raw)
        return jsonify({"ok": True, "filename": name})

    @app.get("/api/ports")
    def api_ports():
        return jsonify({"ports": list_serial_ports()})

    @app.get("/api/node_status")
    def api_node_status():
        snap = _snapshot()
        rt = runtime.status() if runtime is not None else {"mode": "external"}
        return jsonify({
            "ok": True,
            "runtime": rt,
            "link_alive": snap.get("link_alive"),
            "imu_alive": snap.get("imu_alive"),
            "pressure_alive": snap.get("pressure_alive"),
            "position_alive": snap.get("position_alive"),
            "ekf_status_alive": snap.get("ekf_status_alive"),
            "dvl_alive": snap.get("dvl_alive"),
        })

    @app.post("/api/launch")
    def api_launch():
        if runtime is None:
            return jsonify({"ok": False, "error": "Runtime supervisor unavailable"}), 503
        body = request.get_json(silent=True) or {}
        try:
            st = runtime.start_native_sensors(
                xsens_dev=body.get("xsens_dev", "/dev/ttyUSB0"),
                depth_dev=body.get("depth_dev", "/dev/ttyUSB1"),
                connect_a50=bool(body.get("connect_a50", False)),
            )
            return jsonify({"ok": True, "runtime": st})
        except Exception as exc:
            return jsonify({"ok": False, "error": str(exc)}), 409

    @app.post("/api/stop_nodes")
    def api_stop_nodes():
        if runtime is None:
            return jsonify({"ok": False, "error": "Runtime supervisor unavailable"}), 503
        return jsonify({"ok": True, "runtime": runtime.stop()})

    @app.get("/api/debug_state")
    def api_debug_state():
        snap = _snapshot()
        usb_devices = []
        try:
            r = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=1.5)
            usb_devices = [row.strip() for row in r.stdout.splitlines() if row.strip()]
        except Exception:
            pass
        return jsonify({
            **snap,
            "logs": [item["text"] for item in commander.recent_status_texts(50)],
            "status_texts": commander.recent_status_texts(50),
            "usb_devices": usb_devices,
            "runtime": runtime.status() if runtime is not None else None,
            "actuators_enabled": _actuators_enabled(),
            "server_time": time.time(),
        })

    @app.get("/api/autotune/state")
    def api_autotune_state():
        return jsonify({
            "ok": True,
            "supported": False,
            "status": "unavailable",
            "message": "ROS2 AutoTune replaced by ArduSub PID/ACRO tuning parameters via /api/get_params and /api/set_param",
        })

    @app.post("/api/gate/enable")
    def api_gate_enable():
        if gate_bridge is None:
            return jsonify({"ok": False, "error": "Gate bridge unavailable"}), 503
        body = request.get_json(silent=True) or {}
        gate_bridge.set_enabled(bool(body.get("enabled", False)))
        return jsonify({"ok": True, "enabled": gate_bridge.enabled})

    @app.get("/api/camera/status")
    def api_camera_status():
        return jsonify({"ok": True, "data": {
            "connected": False, "streaming": False,
            "message": "Use HTTP/MJPEG stream preview URL or start companion GoPro bridge",
        }})

    @app.post("/api/camera/gopro/start")
    @app.post("/api/camera/gopro/stop")
    @app.post("/api/camera/gopro/settings")
    def api_camera_gopro():
        return jsonify({"ok": False, "error": "GoPro USB daemon runs separately from ArduSub"}), 501

    @app.get("/api/camera/snapshot")
    def api_camera_snapshot():
        return jsonify({"ok": False, "error": "No camera frame available"}), 404

    @app.post("/api/dvl/simulate")
    def api_dvl_simulate():
        return jsonify({
            "ok": False,
            "error": "Use demo_native_dashboard.py or test_dvl_native_chain.py for software DVL simulation",
        }), 501

    @app.post("/api/flash_firmware")
    @app.get("/api/flash_status")
    def api_flash_firmware():
        return jsonify({
            "ok": False,
            "error": "ESP32 firmware flashing is locked out during development; flash manually under hardware supervision",
        }), 403

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14650")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--http-port", type=int, default=8088)
    parser.add_argument("--recording-dir", default="integration/logs/dashboard")
    parser.add_argument("--dvl-status-udp", default="",
                        help="optional UDP host:port to receive DVL status telemetry")
    parser.add_argument("--demo-mode", action="store_true", help="label synthetic input in the operator dashboard")
    parser.add_argument("--enable-control", action="store_true",
                        help="enable ArduSub MAVLink command, mission, and parameter routes")
    parser.add_argument("--allow-software-motor-test", action="store_true",
                        help="allow MAV_CMD_DO_MOTOR_TEST in software/loopback test environments")
    parser.add_argument("--gate-ros", action="store_true",
                        help="subscribe to proposal-only ROS gate status (requires sourced ROS packages)")
    args = parser.parse_args()
    telemetry = DashboardTelemetry(system_id=args.system_id)
    lock = threading.Lock()
    send_lock = threading.Lock()
    recorder = DashboardRecorder(args.recording_dir)
    link = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)

    commander = None
    mission_client = None
    guided_mission = None
    timed_mission = None
    flight_plan = None
    runtime = None
    gate_bridge = None

    if args.enable_control:
        cfg_path = Path(__file__).with_name("vehicle_defaults.json")
        actuators_on = False
        try:
            actuators_on = bool(json.loads(cfg_path.read_text(encoding="utf-8"))["actuators"].get("enabled"))
        except Exception:
            pass
        commander = ArduSubCommander(
            link, send_lock=send_lock, system_id=args.system_id,
            actuators_enabled=actuators_on,
            allow_software_motor_test=args.allow_software_motor_test,
        )
        mission_client = MissionClient(commander)
        guided_mission = GuidedMissionRunner(commander, lambda: _locked_snapshot(telemetry, lock))
        timed_mission = TimedMissionRunner(commander, lambda: _locked_snapshot(telemetry, lock))
        flight_plan = FlightPlanRunner(commander, lambda: _locked_snapshot(telemetry, lock))
        runtime = RuntimeSupervisor(cfg_path)
        gate_bridge = GateGuidanceBridge(commander)

    if args.dvl_status_udp:
        host, port_s = args.dvl_status_udp.rsplit(":", 1)
        dvl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dvl_sock.bind((host, int(port_s)))

        def dvl_loop():
            while True:
                data, _ = dvl_sock.recvfrom(4096)
                try:
                    payload = json.loads(data.decode("utf-8"))
                    with lock:
                        telemetry.ingest_dvl_status(payload)
                except Exception:
                    pass

        threading.Thread(target=dvl_loop, daemon=True).start()

    if args.gate_ros:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from auv_interfaces_ros2.msg import GateGuidance

        class GateReader(Node):
            def __init__(self):
                super().__init__("atal_dashboard_gate_reader")
                self.create_subscription(Bool, "/auv/perception/gate_link_ok", self.on_link, 10)
                self.create_subscription(GateGuidance, "/auv/mission/gate_guidance", self.on_guidance, 10)

            def on_link(self, msg):
                with lock:
                    telemetry.ingest_gate_link(msg.data)

            def on_guidance(self, msg):
                with lock:
                    telemetry.ingest_gate_guidance(msg)

        rclpy.init()
        gate_reader = GateReader()
        threading.Thread(target=rclpy.spin, args=(gate_reader,), daemon=True).start()

    def read_loop():
        while True:
            msg = link.recv_match(blocking=True, timeout=1.0)
            if msg is not None and msg.get_type() != "BAD_DATA":
                with lock:
                    telemetry.ingest(msg)
                if commander is not None:
                    commander.handle_message(msg)

    threading.Thread(target=read_loop, daemon=True).start()

    def record_loop():
        while True:
            with lock:
                snapshot = telemetry.snapshot()
            recorder.sample(snapshot)
            if gate_bridge is not None and gate_bridge.enabled:
                gate_bridge.step(snapshot)
            time.sleep(0.1)

    threading.Thread(target=record_loop, daemon=True).start()
    make_app(
        telemetry, lock, recorder, demo_mode=args.demo_mode,
        commander=commander, mission_client=mission_client,
        guided_mission=guided_mission, timed_mission=timed_mission,
        flight_plan=flight_plan, runtime=runtime, gate_bridge=gate_bridge,
    ).run(host="127.0.0.1", port=args.http_port, debug=False)


def _locked_snapshot(telemetry, lock):
    with lock:
        return telemetry.snapshot()


if __name__ == "__main__":
    main()
