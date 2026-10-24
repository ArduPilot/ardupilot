#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Exercise Tracker's logger chunk validation with interleaved messages."""

import sys
import unittest

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from antennatracker import AutoTestTracker  # noqa: E402
from vehicle_test_suite import NotAchievedException  # noqa: E402


class LoggerChunksTest(unittest.TestCase):
    def run_chunks(self, change=None):
        messages = []

        def log_text(text):
            # Logger's first MSG includes its source; continuations do not.
            text = 'SRC=250/250:' + text
            messages.extend(SimpleNamespace(Seq=sequence, Message=text[offset:offset + 64])
                            for sequence, offset in enumerate(range(0, len(text), 64)))

        def reader():
            if change is not None:
                change(messages)
            return SimpleNamespace(recv_match=lambda **kwargs: messages.pop(0) if messages else None)

        suite = SimpleNamespace(assert_parameter_value=Mock(), send_statustext=log_text,
                                delay_sim_time=Mock(), dfreader_for_current_onboard_log=reader,
                                reboot_sitl=Mock(), progress=Mock())
        AutoTestTracker.LoggerMsgChunks(suite)

    def test_complete_message(self):
        self.run_chunks()

    def test_standalone_messages_do_not_reset_chunk_sequence(self):
        def interleave(messages):
            messages.insert(2, SimpleNamespace(Seq=0, Message='AHRS: EKF3 active'))
        self.run_chunks(interleave)

    def test_missing_chunk_is_rejected(self):
        with self.assertRaises(NotAchievedException):
            self.run_chunks(lambda messages: messages.pop(2))

    def test_wrong_sequence_is_rejected(self):
        def reorder(messages):
            messages[2].Seq = 7
        with self.assertRaises(NotAchievedException):
            self.run_chunks(reorder)

    def test_corrupt_chunk_is_rejected(self):
        def corrupt(messages):
            messages[2].Message = 'wrong contents'
        with self.assertRaises(NotAchievedException):
            self.run_chunks(corrupt)


if __name__ == '__main__':
    unittest.main()
