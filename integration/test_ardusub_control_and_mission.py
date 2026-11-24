"""Unit & API integration tests for ArduSubCommander, MissionClient, runners, runtime, and dashboard control routes."""

import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

os.environ["MAVLINK20"] = "1"
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "modules/mavlink"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from pymavlink import mavutil  # noqa: E402

from ardusub_control import ArduSubCommander  # noqa: E402
from atal_runtime import RuntimeSupervisor, validate_serial_ownership  # noqa: E402
from dashboard_api import make_app  # noqa: E402
from dashboard_recorder import DashboardRecorder  # noqa: E402
from dashboard_telemetry import DashboardTelemetry  # noqa: E402
from flight_plan_runner import FlightPlanRunner, TimedMissionRunner  # noqa: E402
from gate_bridge import GateGuidanceBridge  # noqa: E402
from mission_client import (  # noqa: E402
    GuidedMissionRunner,
    MissionClient,
    local_ned_to_global,
    ros_waypoints_to_mission_items,
)
from vehicle_dynamics_sim import UnderwaterDynamicsSim  # noqa: E402


class FakeMsg:
    def __init__(self, kind, src_sys=1, src_comp=1, **fields):
        self._kind = kind
        self._sys = src_sys
        self._comp = src_comp
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._sys

    def get_srcComponent(self):
        return self._comp


class FakeMav:
    def __init__(self, commander_ref):
        self.sent = []
        self._cmd_ref = commander_ref
        self._stored_items = []

    def command_long_send(self, sys_id, comp_id, cmd, conf, p1, p2, p3, p4, p5, p6, p7):
        self.sent.append(("COMMAND_LONG", cmd, p1, p2, p3, p4))
        c = self._cmd_ref()
        if c is not None:
            threading.Timer(0.01, lambda: c.handle_message(
                FakeMsg("COMMAND_ACK", command=cmd, result=0, progress=0, result_param2=0)
            )).start()

    def manual_control_send(self, sys_id, x, y, z, r, buttons):
        self.sent.append(("MANUAL_CONTROL", x, y, z, r, buttons))

    def rc_channels_override_send(self, sys_id, comp_id, *ch):
        self.sent.append(("RC_CHANNELS_OVERRIDE", ch))

    def set_position_target_local_ned_send(self, t, sys_id, comp_id, frame, mask,
                                           x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate):
        self.sent.append(("SET_POSITION_TARGET_LOCAL_NED", frame, mask, x, y, z, vx, vy, vz, yaw, yaw_rate))

    def set_attitude_target_send(self, t, sys_id, comp_id, mask, q, br, bp, by, thrust):
        self.sent.append(("SET_ATTITUDE_TARGET", mask, tuple(q), thrust))

    def set_gps_global_origin_send(self, sys_id, lat, lon, alt):
        self.sent.append(("SET_GPS_GLOBAL_ORIGIN", lat, lon, alt))

    def param_request_list_send(self, sys_id, comp_id):
        self.sent.append(("PARAM_REQUEST_LIST",))
        c = self._cmd_ref()
        if c is not None:
            for idx, (name, val) in enumerate((("ATC_RAT_RLL_P", 0.15), ("WP_SPD", 0.4), ("MOT_1_DIRECTION", -1.0))):
                c.handle_message(FakeMsg("PARAM_VALUE", param_id=name.encode(), param_value=val,
                                         param_type=9, param_index=idx, param_count=3))

    def param_request_read_send(self, sys_id, comp_id, name_bytes, idx):
        self.sent.append(("PARAM_REQUEST_READ", name_bytes))

    def param_set_send(self, sys_id, comp_id, name_bytes, val, ptype):
        name = name_bytes.split(b"\x00", 1)[0].decode("ascii")
        self.sent.append(("PARAM_SET", name, val))
        c = self._cmd_ref()
        if c is not None:
            threading.Timer(0.01, lambda: c.handle_message(
                FakeMsg("PARAM_VALUE", param_id=name_bytes, param_value=val,
                        param_type=ptype, param_index=0, param_count=1)
            )).start()

    def mission_clear_all_send(self, sys_id, comp_id, mtype):
        self._stored_items.clear()
        c = self._cmd_ref()
        if c is not None:
            threading.Timer(0.01, lambda: c.handle_message(FakeMsg("MISSION_ACK", type=0))).start()

    def mission_count_send(self, sys_id, comp_id, count, mtype):
        self._stored_items = [None] * count
        c = self._cmd_ref()
        if c is not None and count > 0:
            threading.Timer(0.01, lambda: c.handle_message(FakeMsg("MISSION_REQUEST_INT", seq=0))).start()

    def mission_item_int_send(self, sys_id, comp_id, seq, frame, cmd, cur, auto,
                              p1, p2, p3, p4, x, y, z, mtype):
        self._stored_items[seq] = dict(seq=seq, frame=frame, command=cmd,
                                       param1=p1, param2=p2, param3=p3, param4=p4, x=x, y=y, z=z)
        c = self._cmd_ref()
        if c is not None:
            if seq + 1 < len(self._stored_items):
                threading.Timer(0.005, lambda s=seq + 1: c.handle_message(
                    FakeMsg("MISSION_REQUEST_INT", seq=s)
                )).start()
            else:
                threading.Timer(0.005, lambda: c.handle_message(FakeMsg("MISSION_ACK", type=0))).start()

    def mission_request_list_send(self, sys_id, comp_id, mtype):
        c = self._cmd_ref()
        if c is not None:
            threading.Timer(0.01, lambda: c.handle_message(
                FakeMsg("MISSION_COUNT", count=len(self._stored_items))
            )).start()

    def mission_request_int_send(self, sys_id, comp_id, seq, mtype):
        c = self._cmd_ref()
        if c is not None:
            item = self._stored_items[seq]
            threading.Timer(0.005, lambda it=item: c.handle_message(FakeMsg("MISSION_ITEM_INT", **it))).start()

    def mission_ack_send(self, sys_id, comp_id, ack_type, mtype):
        self.sent.append(("MISSION_ACK", ack_type))


class FakeConn:
    def __init__(self):
        self.cmd = None
        self.mav = FakeMav(lambda: self.cmd)


class ArduSubControlAndMissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="atal-test-")
        self.conn = FakeConn()
        self.cmd = ArduSubCommander(
            self.conn, profiles_dir=Path(self.tmp.name) / "profiles",
            actuators_enabled=False, allow_software_motor_test=False,
        )
        self.conn.cmd = self.cmd

    def tearDown(self):
        self.cmd.close()
        self.tmp.cleanup()

    def test_mode_arm_kill_and_manual_deadman_watchdog(self):
        self.assertFalse(self.cmd.set_mode("NOT_A_MODE")["ok"])
        self.assertTrue(self.cmd.set_mode("GUIDED")["ok"])
        self.assertTrue(self.cmd.arm(True)["ok"])
        self.assertTrue(self.cmd.motor_test(1, 1600)["ok"] is False)  # actuators disabled

        self.cmd.set_manual_control(600, -200, 500, 100, lease_s=0.12)
        time.sleep(0.25)
        manuals = [m for m in self.conn.mav.sent if m[0] == "MANUAL_CONTROL"]
        self.assertGreaterEqual(len(manuals), 2)
        self.assertEqual(manuals[-1], ("MANUAL_CONTROL", 0, 0, 500, 0, 0))

        self.assertTrue(self.cmd.kill_all()["ok"])
        last_cmd = [m for m in self.conn.mav.sent if m[0] == "COMMAND_LONG"][-1]
        self.assertEqual(last_cmd[1], mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        self.assertEqual((last_cmd[2], last_cmd[3]), (0.0, 21196.0))

    def test_params_and_profiles_roundtrip(self):
        params = self.cmd.refresh_params(timeout_s=1.0)
        self.assertIn("ATC_RAT_RLL_P", params)
        self.assertTrue(self.cmd.set_param("ATC_RAT_RLL_P", 0.22)["ok"])
        self.assertTrue(self.cmd.save_profile("pool_test")["ok"])
        profiles = self.cmd.list_profiles()
        self.assertEqual([p["name"] for p in profiles], ["pool_test"])
        self.assertTrue(self.cmd.load_profile("pool_test")["ok"])
        self.assertTrue(self.cmd.delete_profile("pool_test")["ok"])
        self.assertEqual(self.cmd.list_profiles(), [])

    def test_mission_conversion_upload_and_readback(self):
        wps = [
            {"north": 2.0, "east": 1.0, "depth": 1.5, "yaw": 90.0,
             "max_vel": 0.4, "tolerance": 0.3, "hold_sec": 2.0},
            {"north": 0.0, "east": 0.0, "depth": 0.5, "yaw": 0.0,
             "max_vel": 0.4, "tolerance": 0.25, "hold_sec": 1.0},
        ]
        items = ros_waypoints_to_mission_items(wps, 26.5, 80.2)
        self.assertEqual(items[0].seq, 0)
        wp_items = [it for it in items if it.seq > 0 and it.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT]
        self.assertEqual(len(wp_items), 2)
        self.assertAlmostEqual(wp_items[0].z, -1.5)
        lat0, lon0 = local_ned_to_global(2.0, 1.0, 26.5, 80.2)
        self.assertEqual(wp_items[0].x, int(round(lat0 * 1e7)))

        mc = MissionClient(self.cmd)
        up = mc.upload_items(items)
        self.assertTrue(up["ok"])
        rb = mc.readback()
        self.assertTrue(rb["ok"])
        self.assertEqual(rb["count"], len(items))
        self.assertAlmostEqual(rb["items"][2]["z"], -1.5)

    def test_gate_guidance_bridge_gating(self):
        bridge = GateGuidanceBridge(self.cmd)
        base = {
            "link_alive": True, "imu_alive": True, "armed_known": True, "armed": True,
            "vehicle_mode": "GUIDED", "yaw": 0.0,
            "gate_link_alive": True, "gate_link_ok": True,
            "gate_guidance_alive": True, "gate_proposal_valid": True,
            "gate_state": "ALIGN", "gate_forward": 0.2, "gate_lateral": -0.5,
            "gate_vertical": 0.1, "gate_yaw": 0.3,
        }
        self.assertFalse(bridge.step(base)["commanded"])
        bridge.set_enabled(True)
        res = bridge.step(base)
        self.assertTrue(res["commanded"])
        self.assertGreater(res["vn"], 0.0)
        self.assertLess(res["ve"], 0.0)
        # Disarming or leaving GUIDED zeroes velocity
        res2 = bridge.step(dict(base, vehicle_mode="ALT_HOLD"))
        self.assertFalse(res2["commanded"])
        last_pos = [m for m in self.conn.mav.sent if m[0] == "SET_POSITION_TARGET_LOCAL_NED"][-1]
        self.assertEqual((last_pos[6], last_pos[7], last_pos[8]), (0.0, 0.0, 0.0))

    def test_runtime_serial_ownership_and_dynamics_sim(self):
        validate_serial_ownership({"xsens": "/dev/null", "depth": "/dev/zero"})
        with self.assertRaises(ValueError):
            validate_serial_ownership({"xsens": "/dev/null", "depth": "/dev/null"})
        sup = RuntimeSupervisor()
        self.assertEqual(sup.status()["mode"], "stopped")

        sim = UnderwaterDynamicsSim()
        start_depth = sim.pos_ned[2]
        # Drive vertical thrusters downward (AP Vectored-6DOF vertical +1 -> PWM < 1500 because MOT_n_DIR=-1,
        # actually row[3]=+1 with thrust=(1500-pwm)/200; pwm=1350 -> thrust=+0.75 -> up; pwm=1650 -> down)
        for _ in range(20):
            sim.step([1500, 1500, 1500, 1500, 1350, 1350, 1350, 1350], True, 0.05)
        self.assertGreater(sim.pos_ned[2], start_depth + 0.05)
        self.assertTrue(sim.depth_line().startswith(b"DEPTH:"))

    def test_interactive_dashboard_api_endpoints(self):
        tel = DashboardTelemetry()
        tel.ingest(FakeMsg("HEARTBEAT", base_mode=129, custom_mode=4))
        tel.ingest(FakeMsg("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.0,
                           rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))
        tel.ingest(FakeMsg("LOCAL_POSITION_NED", x=0.0, y=0.0, z=0.5, vx=0.0, vy=0.0, vz=0.0))
        tel.ingest_dvl_status({"type": "atal_dvl_status", "dvl_locked": True,
                               "dvl_altitude": 2.0, "dvl_fom": 0.04})
        lock = threading.Lock()
        rec = DashboardRecorder(Path(self.tmp.name) / "rec")
        mc = MissionClient(self.cmd)
        gm = GuidedMissionRunner(self.cmd, lambda: tel.snapshot())
        tm = TimedMissionRunner(self.cmd, lambda: tel.snapshot(),
                                library_dir=Path(self.tmp.name) / "tm")
        fp = FlightPlanRunner(self.cmd, lambda: tel.snapshot(),
                              log_dir=Path(self.tmp.name) / "fp")
        gb = GateGuidanceBridge(self.cmd)
        app = make_app(
            tel, lock, recorder=rec, commander=self.cmd, mission_client=mc,
            guided_mission=gm, timed_mission=tm, flight_plan=fp,
            runtime=RuntimeSupervisor(), gate_bridge=gb,
        )
        client = app.test_client()
        try:
            self.assertEqual(client.get("/classic").status_code, 200)
            self.assertIn(b'data-atal-control="1"', client.get("/classic").data)
            self.assertEqual(client.get("/debug").status_code, 200)
            self.assertFalse(client.get("/api/autotune/state").json["supported"])
            self.assertTrue(client.get("/api/state").json["dvl_locked"])
            self.assertTrue(client.post("/api/set_mode", json={"mode": "ALT_HOLD"}).json["ok"])
            self.assertTrue(client.post("/api/arm", json={"arm": True}).json["ok"])
            self.assertTrue(client.get("/api/get_params").json["count"] >= 3)
            self.assertEqual(client.post("/api/motor_test_pwm", json={"motor": 1, "pwm": 1600}).status_code, 403)
            self.assertEqual(client.post("/api/flight_plan/arm_untethered", json={}).status_code, 403)
            self.assertEqual(client.post("/api/flash_firmware", json={}).status_code, 403)
            wps = [{"north": 1.0, "east": 0.5, "depth": 0.5, "yaw": 0.0,
                    "max_vel": 0.3, "tolerance": 0.3, "hold_sec": 0.1}]
            up = client.post("/api/mission/upload", json={"wps": wps})
            self.assertTrue(up.json["ok"])
            self.assertEqual(up.json["readback_count"], 4)
            self.assertTrue(client.post("/api/mission/start", json={"wps": wps}).json["ok"])
            self.assertTrue(client.post("/api/mission/abort").json["ok"])
            tm_mission = {
                "version": 2, "name": "Smoke", "repeat": 1, "max_time_s": 10.0,
                "blocks": [{"type": "target", "label": "Hold", "roll": 0, "pitch": 0,
                            "yaw": 0, "depth": 0.5, "roll_tol": 5, "pitch_tol": 5,
                            "yaw_tol": 10, "depth_tol": 0.2, "stable_s": 0.2,
                            "max_time_s": 5.0, "timeout_action": "continue"}],
            }
            saved = client.post("/api/timed-missions/library", json={"mission": tm_mission})
            self.assertTrue(saved.json["ok"])
            self.assertTrue(client.post("/api/timed-missions/start", json={"mission": tm_mission}).json["ok"])
            self.assertTrue(client.post("/api/timed-missions/abort").json["ok"])
            self.assertTrue(client.post("/api/kill_all").json["ok"])
        finally:
            gm.close()
            tm.close()


if __name__ == "__main__":
    unittest.main()
