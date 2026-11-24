# AP_FLAKE8_CLEAN
"""Check local CSV recording and its HTTP controls without a vehicle."""

import csv
import tempfile
import threading
import unittest
from pathlib import Path

from dashboard_api import make_app
from dashboard_recorder import DashboardRecorder
from dashboard_telemetry import DashboardTelemetry


class DashboardRecorderTests(unittest.TestCase):
    def test_csv_preserves_unknown_state_and_clears_stale_pwm(self):
        with tempfile.TemporaryDirectory(prefix="atal-dashboard-recording-") as directory:
            recorder = DashboardRecorder(directory)
            status = recorder.start()
            self.assertTrue(status["active"])
            self.assertEqual(recorder.start()["file"], status["file"])
            recorder.sample({"link_alive": False, "armed_known": False, "armed": False,
                             "imu_alive": False, "roll": 42.0, "pressure_alive": False,
                             "pressure_mbar": 1025.5, "position_alive": False, "depth_m": 3.0,
                             "mission_alive": False, "mission_state": "Active", "mission_seq": 2,
                             "ekf_status_alive": False, "ekf_flags": 19, "ekf_absolute_position_ok": True,
                             "gate_link_alive": False, "gate_link_ok": True,
                             "gate_guidance_alive": False, "gate_proposal_valid": True,
                             "gate_forward": 0.1,
                             "output_alive": False, "pwm": [1500] * 8}, now=1000.0)
            recorder.sample({"link_alive": True, "armed_known": True, "armed": True,
                             "imu_alive": True, "pressure_alive": True, "position_alive": True,
                             "pressure_mbar": 1025.5, "output_alive": True,
                             "mission_alive": True, "mission_state": "Active", "mission_seq": 2,
                             "ekf_status_alive": True, "ekf_flags": 515, "ekf_absolute_position_ok": False,
                             "mission_total": 4, "mission_mode": "Mission mode",
                             "gate_link_alive": True, "gate_link_ok": True,
                             "gate_guidance_alive": True, "gate_state": "ALIGN",
                             "gate_proposal_valid": True, "gate_forward": 0.0,
                             "gate_lateral": -0.05, "gate_vertical": 0.0, "gate_yaw": -0.1,
                             "pwm": [1400, 1600] + [1500] * 6}, now=1000.1)
            stopped = recorder.stop()
            self.assertFalse(stopped["active"])
            self.assertEqual(stopped["rows"], 2)
            with (Path(directory) / stopped["file"]).open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["armed_known"], "False")
            self.assertEqual(rows[0]["armed"], "")
            self.assertEqual(rows[0]["vehicle_mode"], "")
            self.assertEqual(rows[0]["roll"], "")
            self.assertEqual(rows[0]["pressure_mbar"], "")
            self.assertEqual(rows[0]["depth_m"], "")
            self.assertEqual(rows[0]["pwm_1"], "")
            self.assertEqual(rows[0]["mission_state"], "")
            self.assertEqual(rows[0]["ekf_flags"], "")
            self.assertEqual(rows[0]["gate_forward"], "")
            self.assertEqual(rows[0]["gate_link_ok"], "")
            self.assertEqual(rows[1]["pwm_1"], "1400")
            self.assertEqual(rows[1]["pwm_8"], "1500")
            self.assertEqual((rows[1]["mission_state"], rows[1]["mission_seq"], rows[1]["mission_total"]),
                             ("Active", "2", "4"))
            self.assertEqual((rows[1]["ekf_flags"], rows[1]["ekf_absolute_position_ok"]), ("515", "False"))
            self.assertEqual((rows[1]["gate_state"], rows[1]["gate_lateral"]), ("ALIGN", "-0.05"))

    def test_http_start_status_stop(self):
        with tempfile.TemporaryDirectory(prefix="atal-dashboard-recording-") as directory:
            recorder = DashboardRecorder(directory)
            app = make_app(DashboardTelemetry(), threading.Lock(), recorder)
            client = app.test_client()
            self.assertFalse(client.get("/api/recording").json["active"])
            self.assertEqual(client.post("/api/recording/start").status_code, 403)
            headers = {"X-ATAL-Local-Action": "recording"}
            started = client.post("/api/recording/start", headers=headers).json
            self.assertTrue(started["active"])
            self.assertEqual(client.get("/api/recording").json["file"], started["file"])
            download = client.get("/api/recording/download")
            self.assertEqual(download.status_code, 200)
            download.close()
            stopped = client.post("/api/recording/stop", headers=headers).json
            self.assertFalse(stopped["active"])
            self.assertTrue((Path(directory) / stopped["file"]).is_file())


if __name__ == "__main__":
    unittest.main()
