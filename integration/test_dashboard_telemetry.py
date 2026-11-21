import math
import threading
import unittest

from dashboard_api import make_app
from dashboard_telemetry import DashboardTelemetry


class Message:
    def __init__(self, kind, source=1, component=1, **fields):
        self.kind = kind
        self.source = source
        self.component = component
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.source

    def get_srcComponent(self):
        return self.component


class DashboardTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.time = [0.0]
        self.telemetry = DashboardTelemetry(now=lambda: self.time[0])

    def test_arm_status_and_freshness(self):
        self.telemetry.ingest(Message("HEARTBEAT", source=255, base_mode=128))
        self.telemetry.ingest(Message("HEARTBEAT", component=100, base_mode=128))
        self.assertFalse(self.telemetry.snapshot()["armed_known"])
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=128))
        self.telemetry.ingest(Message("ATTITUDE", roll=0.1, pitch=0, yaw=math.pi / 2,
                                      rollspeed=0.01, pitchspeed=0, yawspeed=0))
        snapshot = self.telemetry.snapshot()
        self.assertTrue(snapshot["armed"])
        self.assertTrue(snapshot["armed_known"])
        self.assertAlmostEqual(snapshot["yaw"], 90)
        self.assertTrue(snapshot["imu_alive"])
        self.time[0] = 4.0
        stale = self.telemetry.snapshot()
        self.assertFalse(stale["link_alive"])
        self.assertFalse(stale["armed_known"])
        self.assertFalse(stale["imu_alive"])

    def test_body_velocity_and_bad_data(self):
        self.telemetry.ingest(Message("ATTITUDE", roll=0, pitch=0, yaw=math.pi / 2,
                                      rollspeed=0, pitchspeed=0, yawspeed=0))
        self.telemetry.ingest(Message("LOCAL_POSITION_NED", x=1, y=2, z=3,
                                      vx=0, vy=1, vz=0.2))
        snapshot = self.telemetry.snapshot()
        self.assertAlmostEqual(snapshot["vel_fwd"], 1)
        self.assertAlmostEqual(snapshot["vel_lat"], 0, places=6)
        self.assertEqual(snapshot["depth_m"], 3)
        self.telemetry.ingest(Message("LOCAL_POSITION_NED", x=1, y=2, z=float("nan"),
                                      vx=3, vy=4, vz=5))
        self.assertEqual(self.telemetry.snapshot()["depth_m"], 3)

    def test_api_is_read_only(self):
        client = make_app(self.telemetry, threading.Lock()).test_client()
        self.assertEqual(client.get("/api/state").json["source"], "ardusub_mavlink")
        self.assertEqual(client.post("/api/arm").status_code, 404)
        self.assertEqual(client.post("/api/state").status_code, 405)


if __name__ == "__main__":
    unittest.main()
