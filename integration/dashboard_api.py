"""Local, read-only dashboard telemetry API for ArduSub MAVLink.

Run from the repository root with:
python3 integration/dashboard_api.py --mavlink udpin:127.0.0.1:14650
"""

import argparse
import threading

from flask import Flask, jsonify
from pymavlink import mavutil

from dashboard_telemetry import DashboardTelemetry


def make_app(telemetry, lock):
    app = Flask(__name__)

    @app.get("/api/state")
    def state():
        with lock:
            return jsonify(telemetry.snapshot())

    @app.get("/api/health")
    def health():
        with lock:
            snapshot = telemetry.snapshot()
        return jsonify({key: snapshot[key] for key in (
            "link_alive", "imu_alive", "pressure_alive", "armed_known", "source"
        )})

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
