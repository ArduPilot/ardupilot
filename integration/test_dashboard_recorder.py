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
            recorder.sample({"link_alive": False, "armed_known": False, "armed": None,
                             "output_alive": False, "pwm": [1500] * 8}, now=1000.0)
            recorder.sample({"link_alive": True, "armed_known": True, "armed": True,
                             "output_alive": True, "pwm": [1400, 1600] + [1500] * 6}, now=1000.1)
            stopped = recorder.stop()
            self.assertFalse(stopped["active"])
            self.assertEqual(stopped["rows"], 2)
            with (Path(directory) / stopped["file"]).open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["armed_known"], "False")
            self.assertEqual(rows[0]["armed"], "")
            self.assertEqual(rows[0]["pwm_1"], "")
            self.assertEqual(rows[1]["pwm_1"], "1400")
            self.assertEqual(rows[1]["pwm_8"], "1500")

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
