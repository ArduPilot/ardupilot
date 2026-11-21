"""Local, read-only dashboard telemetry API for ArduSub MAVLink.

Run from the repository root with:
python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650
"""

import argparse
import json
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, send_file
from pymavlink import mavutil

from dashboard_telemetry import DashboardTelemetry


def make_app(telemetry, lock):
    app = Flask(__name__)

    @app.get("/")
    def dashboard():
        return send_file(Path(__file__).with_name("dashboard_view.html"))

    @app.get("/api/state")
    def state():
        with lock:
            return jsonify(telemetry.snapshot())

    @app.get("/api/health")
    def health():
        with lock:
            snapshot = telemetry.snapshot()
        return jsonify({key: snapshot[key] for key in (
            "link_alive", "imu_alive", "pressure_alive", "position_alive", "armed_known", "source"
        )})

    @app.get("/api/stream")
    def stream():
        # The ROS dashboard already consumes this SSE route. Keep it read only:
        # observing state must not become a vehicle deadman or command signal.
        def events():
            while True:
                with lock:
                    snapshot = telemetry.snapshot()
                yield f"data: {json.dumps(snapshot)}\n\n"
                time.sleep(0.1)

        response = Response(events(), mimetype="text/event-stream")
        response.headers["Cache-Control"] = "no-cache, no-transform"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14650")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--http-port", type=int, default=8088)
    args = parser.parse_args()
    telemetry = DashboardTelemetry(system_id=args.system_id)
    lock = threading.Lock()
    link = mavutil.mavlink_connection(args.mavlink, autoreconnect=True)

    def read_loop():
        while True:
            msg = link.recv_match(blocking=True, timeout=1.0)
            if msg is not None and msg.get_type() != "BAD_DATA":
                with lock:
                    telemetry.ingest(msg)

    threading.Thread(target=read_loop, daemon=True).start()
    make_app(telemetry, lock).run(host="127.0.0.1", port=args.http_port, debug=False)


if __name__ == "__main__":
    main()
