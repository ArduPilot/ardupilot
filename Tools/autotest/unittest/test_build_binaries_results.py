"""Tests for reporting failed binary builds in autotest results.

AP_FLAKE8_CLEAN
"""

import importlib.util
import os
import sqlite3
import subprocess
import sys
import tempfile
import unittest

from pathlib import Path
from unittest.mock import Mock
from unittest.mock import patch

scripts_path = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(scripts_path))
try:
    build_script_spec = importlib.util.spec_from_file_location(
        "build_binaries", scripts_path / "build_binaries.py"
    )
    build_script = importlib.util.module_from_spec(build_script_spec)
    build_script_spec.loader.exec_module(build_script)
finally:
    sys.path.pop(0)

helper_path = Path(__file__).resolve().parents[1] / "build_binaries_results.py"
helper_spec = importlib.util.spec_from_file_location("build_binaries_results", helper_path)
helper_module = importlib.util.module_from_spec(helper_spec)
helper_spec.loader.exec_module(helper_module)
add_build_failure_results = helper_module.add_build_failure_results


class TestBuildBinaryFailureResults(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.database_path = os.path.join(self.temp_dir.name, "build-history.sqlite")
        self.connection = sqlite3.connect(self.database_path)
        self.addCleanup(self.connection.close)
        self.connection.execute(
            "CREATE TABLE build (hash text, tag text, vehicle text, board text, "
            "frame text, text integer, data integer, bss integer, start_time real, duration real)"
        )
        self.connection.execute(
            "CREATE TABLE run (hash text, tag text, start_time real, duration real)"
        )
        self.results = []

    def add_run(self, build_hash, tag, start_time):
        self.connection.execute(
            "INSERT INTO run VALUES (?, ?, ?, ?)",
            (build_hash, tag, start_time, 10),
        )
        self.connection.commit()

    def add_build(self, build_hash, tag, vehicle, board, frame, text, start_time, duration):
        self.connection.execute(
            "INSERT INTO build VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (build_hash, tag, vehicle, board, frame, text, None, None, start_time, duration),
        )
        self.connection.commit()

    def add(self, name, result, elapsed):
        self.results.append((name, result, elapsed))

    def test_reports_only_failed_latest_builds_from_this_run(self):
        self.add_run("current-hash", "latest", 11)
        self.add_run("old-hash", "latest", 10)
        self.add_run("current-hash", "stable", 12)
        self.add_build("current-hash", "latest", "Copter", "fmuv3", None, None, 12, 2.5)
        self.add_build("current-hash", "latest", "Copter", "fmuv3", None, None, 13, 3.5)
        self.add_build("current-hash", "latest", "Plane", "Pixhawk1", "quad", None, 13, 4.0)
        self.add_build("current-hash", "latest", "Rover", "sitl", None, 100, 13, 1.0)
        self.add_build("current-hash", "stable", "Sub", "Pixhawk1", None, None, 13, 1.0)
        self.add_build("old-hash", "latest", "Blimp", "sitl", None, None, 10, 1.0)
        self.add_build("different-hash", "latest", "Tracker", "sitl", None, None, 14, 1.0)

        add_build_failure_results(self, self.database_path, 11)

        self.assertEqual(
            self.results,
            [
                ("Build failure: Copter fmuv3", '<span class="failed-text">FAILED</span>', 3.5),
                ("Build failure: Plane Pixhawk1 quad", '<span class="failed-text">FAILED</span>', 4.0),
            ],
        )

    def test_no_current_run_does_not_report_old_builds(self):
        self.add_build("old-hash", "latest", "Copter", "fmuv3", None, None, 10, 1.0)

        add_build_failure_results(self, self.database_path, 11)

        self.assertEqual(self.results, [])

    def test_missing_database_does_not_add_results(self):
        missing_path = os.path.join(self.temp_dir.name, "missing.sqlite")

        add_build_failure_results(self, missing_path, 11)

        self.assertEqual(self.results, [])

    def test_database_without_history_tables_does_not_add_results(self):
        empty_path = os.path.join(self.temp_dir.name, "empty.sqlite")
        sqlite3.connect(empty_path).close()

        add_build_failure_results(self, empty_path, 11)

        self.assertEqual(self.results, [])

    def test_configure_failure_is_recorded_in_build_history(self):
        builder = build_script.build_binaries(["latest"])
        builder.error_strings = []
        builder.binaries = self.temp_dir.name
        builder.hdate_ym = "2026-10"
        builder.hdate_ymdhm = "2026-10-26-12-00"
        builder.buildroot = os.path.join(self.temp_dir.name, "buildroot")
        builder.progress = Mock()
        builder.checkout = Mock(return_value=True)
        builder.skip_build = Mock(return_value=False)
        builder.skip_frame = Mock(return_value=False)
        builder.skip_board_waf = Mock(return_value=False)
        builder.run_git_update_submodules = Mock()
        builder.remove_tmpdir = Mock()
        builder.run_git = Mock(return_value="build-hash\n")
        builder.run_waf = Mock(side_effect=subprocess.CalledProcessError(1, "waf"))
        builder.history = Mock()

        with patch.object(build_script.time, "time", side_effect=[90, 100, 102]):
            built_any = builder._build_vehicle_in_staging(
                "latest", "Copter", ["testboard"], "Copter", "arducopter", self.temp_dir.name
            )

        self.assertFalse(built_any)
        self.assertEqual(builder.error_strings, ["Failed configure of Copter testboard latest"])
        builder.history.record_build.assert_called_once_with(
            "build-hash", "latest", "Copter", "testboard", None, None, 100, 2
        )

    def test_latest_checkout_failure_is_recorded_in_build_history(self):
        builder = build_script.build_binaries(["latest"])
        builder.error_strings = []
        builder.progress = Mock()
        builder.checkout = Mock(return_value=False)
        builder.run_git = Mock(return_value="master-hash\n")
        builder.history = Mock()

        with patch.object(build_script.time, "time", side_effect=[100, 102]):
            built_any = builder._build_vehicle_in_staging(
                "latest", "Copter", ["testboard"], "Copter", "arducopter", self.temp_dir.name
            )

        self.assertFalse(built_any)
        self.assertEqual(builder.error_strings, ["Failed checkout of Copter testboard latest None"])
        builder.run_git.assert_called_once_with(["rev-parse", "master"])
        builder.history.record_build.assert_called_once_with(
            "master-hash", "latest", "Copter", "testboard", None, None, 100, 2
        )

    def test_build_failure_is_recorded_in_build_history(self):
        builder = build_script.build_binaries(["latest"])
        builder.error_strings = []
        builder.binaries = self.temp_dir.name
        builder.hdate_ym = "2026-10"
        builder.hdate_ymdhm = "2026-10-26-12-00"
        builder.buildroot = os.path.join(self.temp_dir.name, "buildroot")
        builder.progress = Mock()
        builder.checkout = Mock(return_value=True)
        builder.skip_build = Mock(return_value=False)
        builder.skip_frame = Mock(return_value=False)
        builder.skip_board_waf = Mock(return_value=False)
        builder.run_git_update_submodules = Mock()
        builder.remove_tmpdir = Mock()
        builder.run_git = Mock(return_value="build-hash\n")
        builder.run_waf = Mock(side_effect=[None, subprocess.CalledProcessError(1, "waf")])
        builder.history = Mock()

        with patch.object(build_script.time, "time", side_effect=[90, 100, 101, 103]):
            built_any = builder._build_vehicle_in_staging(
                "latest", "Copter", ["testboard"], "Copter", "arducopter", self.temp_dir.name
            )

        self.assertFalse(built_any)
        self.assertEqual(builder.error_strings, ["Failed build of Copter testboard latest"])
        builder.history.record_build.assert_called_once_with(
            "build-hash", "latest", "Copter", "testboard", None, None, 100, 3
        )


if __name__ == "__main__":
    unittest.main()
