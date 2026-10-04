"""Tests for Tools/autotest/run_in_terminal_window.sh."""

import os
import pathlib
import subprocess
import time

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = REPO_ROOT / "Tools" / "autotest" / "run_in_terminal_window.sh"

# The script chooses how to launch by probing the environment. Clear
# everything it looks at, so it takes the headless branch under test instead
# of opening a terminal on the machine running the tests.
TERMINAL_VARS = ("SITL_RITW_TERMINAL", "TMUX", "DISPLAY", "STY", "ZELLIJ")


def run_headless(name, command, tmpdir=None):
    """Run the script with no terminal available and return the result."""
    env = {k: v for k, v in os.environ.items() if k not in TERMINAL_VARS}
    if tmpdir is None:
        env.pop("TMPDIR", None)
    else:
        env["TMPDIR"] = str(tmpdir)
    return subprocess.run(
        [str(SCRIPT), name] + command,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
    )


def wait_for(path, timeout=10):
    """The script backgrounds the command, so the log appears asynchronously."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if path.exists() and path.stat().st_size > 0:
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def vehicle_name():
    """A name unique to this process, so a stale log cannot mask a result."""
    name = "RiTWTestVehicle%u" % os.getpid()
    yield name
    # A version that ignores TMPDIR writes here; do not leave that behind.
    pathlib.Path("/tmp", name + ".log").unlink(missing_ok=True)


def test_console_log_goes_to_tmpdir(tmp_path, vehicle_name):
    """TMPDIR decides where the console log is written.

    A caller running several vehicles, or sharing a machine with another
    user, needs to control where these logs go.
    """
    result = run_headless(vehicle_name, ["/bin/echo", "hello"], tmpdir=tmp_path)
    assert result.returncode == 0

    log = tmp_path / (vehicle_name + ".log")
    assert wait_for(log), "no console log in TMPDIR; stderr: %s" % result.stderr
    assert "hello" in log.read_text()


def test_console_log_falls_back_to_tmp(vehicle_name):
    """With TMPDIR unset the log still lands in /tmp (the current behavior)."""
    result = run_headless(vehicle_name, ["/bin/echo", "hello"])
    assert result.returncode == 0

    log = pathlib.Path("/tmp", vehicle_name + ".log")
    assert wait_for(log), "no console log in /tmp; stderr: %s" % result.stderr
    assert "hello" in log.read_text()
