#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Unit tests for the Renode mission test helpers."""

import importlib.util

from pathlib import Path
from types import SimpleNamespace

MODULE_PATH = Path(__file__).resolve().parent / 'test_mission.py'
SPEC = importlib.util.spec_from_file_location('renode_test_mission', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
test_mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(test_mission)


class FakeMAV:
    def __init__(self):
        self.log_list_requests = 0

    def log_request_list_send(self, _system, _component, _start, _end):
        self.log_list_requests += 1


class FakeConnection:
    def __init__(self, entries):
        self.mav = FakeMAV()
        self.target_system = 1
        self.target_component = 1
        self.entries = iter(entries)

    def recv_match(self, **_kwargs):
        return next(self.entries)


def test_latest_log_entry_retries_empty_list(monkeypatch, tmp_path):
    expected = SimpleNamespace(num_logs=1, id=1, last_log_num=1, size=1024)
    connection = FakeConnection([
        SimpleNamespace(num_logs=0),
        expected,
    ])
    process = SimpleNamespace(poll=lambda: None)
    monkeypatch.setattr(test_mission.time, 'sleep', lambda _delay: None)

    result = test_mission.latest_log_entry(
        connection, process, tmp_path / 'renode.log', timeout=1)

    assert result is expected
    assert connection.mav.log_list_requests == 2
