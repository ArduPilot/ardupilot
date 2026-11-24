"""Local ArduSub dashboard: read vehicle telemetry and optionally record CSV.

Run from the repository root with:
python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650
"""

import argparse
import json
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request, send_file
from pymavlink import mavutil

from dashboard_telemetry import DashboardTelemetry
from dashboard_recorder import DashboardRecorder
from mission_queue import validate_ros_waypoints


def make_app(telemetry, lock, recorder=None, demo_mode=False):
    app = Flask(__name__)

    @app.get("/")
    def dashboard():
        return send_file(Path(__file__).with_name("operator_dashboard.html"))

    @app.get("/diagnostics")
    def diagnostics():
        return send_file(Path(__file__).with_name("dashboard_view.html"))

    @app.get("/api/state")
    def state():
        with lock:
            snapshot = telemetry.snapshot()
        snapshot["demo_mode"] = demo_mode
        return jsonify(snapshot)

    @app.get("/api/health")
    def health():
        with lock:
            snapshot = telemetry.snapshot()
        return jsonify({key: snapshot[key] for key in (
            "link_alive", "imu_alive", "pressure_alive", "position_alive", "ekf_status_alive",
            "ekf_attitude_ok", "ekf_velocity_horizontal_ok", "ekf_absolute_position_ok",
            "mission_alive", "battery_alive",
            "gate_link_alive", "gate_guidance_alive", "gate_link_ok", "armed_known", "source"
        )})

    @app.get("/api/stream")
    def stream():
        # The ROS dashboard already consumes this SSE route. Keep it read only:
        # observing state must not become a vehicle deadman or command signal.
        def events():
            while True:
                with lock:
                    snapshot = telemetry.snapshot()
                snapshot["demo_mode"] = demo_mode
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
                        "upload_ready": False, "wps": waypoints})

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

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14650")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--http-port", type=int, default=8088)
    parser.add_argument("--recording-dir", default="integration/logs/dashboard")
    parser.add_argument("--demo-mode", action="store_true", help="label synthetic input in the operator dashboard")
    parser.add_argument("--gate-ros", action="store_true",
                        help="subscribe to proposal-only ROS gate status (requires sourced ROS packages)")
    args = parser.parse_args()
    telemetry = DashboardTelemetry(system_id=args.system_id)
    lock = threading.Lock()
    recorder = DashboardRecorder(args.recording_dir)
    link = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)

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

    threading.Thread(target=read_loop, daemon=True).start()

    def record_loop():
        while True:
            with lock:
                snapshot = telemetry.snapshot()
            recorder.sample(snapshot)
            time.sleep(0.1)

    threading.Thread(target=record_loop, daemon=True).start()
    make_app(telemetry, lock, recorder, demo_mode=args.demo_mode).run(host="127.0.0.1", port=args.http_port, debug=False)


if __name__ == "__main__":
    main()
