import math
import json
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
        self.assertIsNone(self.telemetry.snapshot()["armed"])
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=129, custom_mode=3))
        self.telemetry.ingest(Message("ATTITUDE", roll=0.1, pitch=0, yaw=math.pi / 2,
                                      rollspeed=0.01, pitchspeed=0, yawspeed=0))
        self.telemetry.ingest(Message("SCALED_PRESSURE", press_abs=1025.5, temperature=2340))
        self.telemetry.ingest(Message("SERVO_OUTPUT_RAW", port=0,
                                      **{f"servo{i}_raw": 1500 for i in range(1, 9)}))
        snapshot = self.telemetry.snapshot()
        self.assertTrue(snapshot["armed"])
        self.assertTrue(snapshot["armed_known"])
        self.assertEqual(snapshot["vehicle_mode"], "AUTO")
        self.assertAlmostEqual(snapshot["yaw"], 90)
        self.assertTrue(snapshot["imu_alive"])
        self.assertEqual(snapshot["pressure_mbar"], 1025.5)
        self.assertEqual(snapshot["pwm"][0], 1500)
        self.time[0] = 4.0
        stale = self.telemetry.snapshot()
        self.assertFalse(stale["link_alive"])
        self.assertFalse(stale["armed_known"])
        self.assertIsNone(stale["armed"])
        self.assertIsNone(stale["vehicle_mode"])
        self.assertFalse(stale["imu_alive"])
        self.assertIsNone(stale["roll"])
        self.assertIsNone(stale["imu_hz"])
        self.assertIsNone(stale["pressure_mbar"])
        self.assertIsNone(stale["pwm"][0])

    def test_body_velocity_and_bad_data(self):
        self.telemetry.ingest(Message("ATTITUDE", roll=0, pitch=0, yaw=math.pi / 2,
                                      rollspeed=0, pitchspeed=0, yawspeed=0))
        self.telemetry.ingest(Message("LOCAL_POSITION_NED", x=1, y=2, z=3,
                                      vx=0, vy=1, vz=0.2))
        snapshot = self.telemetry.snapshot()
        self.assertAlmostEqual(snapshot["vel_fwd"], 1)
        self.assertAlmostEqual(snapshot["vel_lat"], 0, places=6)
        self.assertEqual(snapshot["depth_m"], 3)
        self.assertTrue(snapshot["position_alive"])
        self.telemetry.ingest(Message("LOCAL_POSITION_NED", x=1, y=2, z=float("nan"),
                                      vx=3, vy=4, vz=5))
        self.assertEqual(self.telemetry.snapshot()["depth_m"], 3)
        self.time[0] = 2.6
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        self.telemetry.ingest(Message("ATTITUDE", roll=0, pitch=0, yaw=0,
                                      rollspeed=0, pitchspeed=0, yawspeed=0))
        stale = self.telemetry.snapshot()
        self.assertTrue(stale["link_alive"])
        self.assertFalse(stale["position_alive"])
        self.assertIsNone(stale["depth_m"])
        self.assertIsNone(stale["vel_fwd"])

    def test_mode_requires_custom_mode_flag(self):
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0, custom_mode=3))
        self.assertIsNone(self.telemetry.snapshot()["vehicle_mode"])
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=1, custom_mode=99))
        self.assertIsNone(self.telemetry.snapshot()["vehicle_mode"])

    def test_battery_sys_status_requires_fresh_link_and_known_values(self):
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        self.telemetry.ingest(Message("SYS_STATUS", source=255, voltage_battery=16000,
                                      current_battery=125, battery_remaining=60))
        self.assertFalse(self.telemetry.snapshot()["battery_alive"])
        self.telemetry.ingest(Message("SYS_STATUS", voltage_battery=16000,
                                      current_battery=125, battery_remaining=60))
        live = self.telemetry.snapshot()
        self.assertTrue(live["battery_alive"])
        self.assertEqual((live["battery_v"], live["battery_a"], live["battery_percent"]), (16.0, 1.25, 60))
        self.telemetry.ingest(Message("SYS_STATUS", voltage_battery=65535,
                                      current_battery=-1, battery_remaining=-1))
        unknown = self.telemetry.snapshot()
        self.assertTrue(unknown["battery_alive"])
        self.assertIsNone(unknown["battery_v"])
        self.assertIsNone(unknown["battery_a"])
        self.assertIsNone(unknown["battery_percent"])
        self.time[0] = 3.1
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        self.assertFalse(self.telemetry.snapshot()["battery_alive"])

    def test_ekf_flags_are_reported_without_claiming_navigation_readiness(self):
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        self.telemetry.ingest(Message("EKF_STATUS_REPORT", source=255, flags=1 | 2 | 16))
        self.assertFalse(self.telemetry.snapshot()["ekf_status_alive"])
        self.telemetry.ingest(Message("EKF_STATUS_REPORT", flags=1 | 2 | 512))
        state = self.telemetry.snapshot()
        self.assertTrue(state["ekf_status_alive"])
        self.assertTrue(state["ekf_attitude_ok"])
        self.assertTrue(state["ekf_velocity_horizontal_ok"])
        self.assertTrue(state["ekf_predicted_position_ok"])
        self.assertFalse(state["ekf_absolute_position_ok"])
        self.assertFalse(state["ekf_uninitialized"])
        self.telemetry.ingest(Message("EKF_STATUS_REPORT", flags=-1))
        self.assertEqual(self.telemetry.snapshot()["ekf_flags"], 1 | 2 | 512)
        self.time[0] = 3.1
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        stale = self.telemetry.snapshot()
        self.assertFalse(stale["ekf_status_alive"])
        self.assertIsNone(stale["ekf_absolute_position_ok"])

    def test_gate_proposals_require_fresh_link_and_validity(self):
        self.telemetry.ingest_gate_link(True)
        self.telemetry.ingest_gate_guidance(Message("GATE", state="ALIGN", command_valid=True,
                                                   forward=0.1, lateral=-0.05, vertical=0.0, yaw=0.02,
                                                   forward_limit=0.15, lateral_limit=0.1,
                                                   vertical_limit=0.1, yaw_limit=0.2))
        live = self.telemetry.snapshot()
        self.assertTrue(live["gate_proposal_valid"])
        self.assertEqual(live["gate_lateral"], -0.05)
        self.telemetry.ingest_gate_guidance(Message("GATE", state="ALIGN", command_valid=True,
                                                   forward=2.0, lateral=0.0, vertical=0.0, yaw=0.0,
                                                   forward_limit=0.15, lateral_limit=0.1,
                                                   vertical_limit=0.1, yaw_limit=0.2))
        self.assertFalse(self.telemetry.snapshot()["gate_guidance_alive"])
        self.telemetry.ingest_gate_guidance(Message("GATE", state="ALIGN", command_valid=True,
                                                   forward=0.1, lateral=-0.05, vertical=0.0, yaw=0.02,
                                                   forward_limit=0.15, lateral_limit=0.1,
                                                   vertical_limit=0.1, yaw_limit=0.2))
        self.time[0] = 0.6
        stale = self.telemetry.snapshot()
        self.assertIsNone(stale["gate_link_ok"])
        self.assertIsNone(stale["gate_forward"])
        self.telemetry.ingest_gate_link(False)
        self.telemetry.ingest_gate_guidance(Message("GATE", state="SEARCH", command_valid=True,
                                                   forward=0.1, lateral=0.0, vertical=0.0, yaw=0.0,
                                                   forward_limit=0.15, lateral_limit=0.1,
                                                   vertical_limit=0.1, yaw_limit=0.2))
        invalid = self.telemetry.snapshot()
        self.assertFalse(invalid["gate_proposal_valid"])
        self.assertIsNone(invalid["gate_forward"])
        self.telemetry.ingest_gate_link(True)
        self.assertFalse(self.telemetry.snapshot()["gate_guidance_alive"])

    def test_api_is_read_only(self):
        client = make_app(self.telemetry, threading.Lock()).test_client()
        self.assertEqual(client.get("/api/state").json["source"], "ardusub_mavlink")
        self.assertIsNone(client.get("/api/state").json["dvl_locked"])
        self.assertFalse(client.get("/api/health").json["mission_alive"])
        self.assertFalse(client.get("/api/health").json["ekf_status_alive"])
        self.assertFalse(client.get("/api/health").json["gate_link_alive"])
        stream = client.get("/api/stream", buffered=False)
        self.assertEqual(stream.mimetype, "text/event-stream")
        event = next(iter(stream.response)).decode("utf-8")
        self.assertEqual(json.loads(event.removeprefix("data: "))["source"], "ardusub_mavlink")
        stream.close()
        self.assertEqual(client.post("/api/arm").status_code, 404)
        self.assertEqual(client.post("/api/state").status_code, 405)

    def test_mission_current_is_read_only_and_expires(self):
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        self.telemetry.ingest(Message("MISSION_CURRENT", source=255, seq=2, total=4,
                                      mission_state=3, mission_mode=1))
        self.assertFalse(self.telemetry.snapshot()["mission_alive"])
        self.telemetry.ingest(Message("MISSION_CURRENT", seq=2, total=4,
                                      mission_state=3, mission_mode=1))
        live = self.telemetry.snapshot()
        self.assertEqual((live["mission_seq"], live["mission_total"]), (2, 4))
        self.assertEqual((live["mission_state"], live["mission_mode"]), ("Active", "Mission mode"))
        self.time[0] = 3.1
        self.telemetry.ingest(Message("HEARTBEAT", base_mode=0))
        stale = self.telemetry.snapshot()
        self.assertFalse(stale["mission_alive"])
        self.assertIsNone(stale["mission_state"])
        self.assertIsNone(stale["mission_seq"])
        self.telemetry.ingest(Message("MISSION_CURRENT", seq=0, total=65535,
                                      mission_state=1, mission_mode=0))
        empty = self.telemetry.snapshot()
        self.assertEqual(empty["mission_state"], "No mission")
        self.assertIsNone(empty["mission_total"])
        self.assertIsNone(empty["mission_seq"])


if __name__ == "__main__":
    unittest.main()
