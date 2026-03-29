#!/usr/bin/env python3
"""
Unit tests for sitl_mavlink_vision_bridge.py

AP_FLAKE8_CLEAN
"""

import math
import os
import sys
import time
import unittest
from unittest.mock import MagicMock, patch

# ensure the script's own directory is on the path regardless of cwd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sitl_mavlink_vision_bridge import (  # noqa: E402
    build_pose_covariance,
    main,
    parse_args,
    send_landing_target_demo,
    send_vision_demo,
)


class TestParseArgs(unittest.TestCase):
    def test_defaults(self):
        args = parse_args([])
        self.assertEqual(args.connection, 'tcp:127.0.0.1:5760')
        self.assertEqual(args.mode, 'vision')
        self.assertAlmostEqual(args.rate, 20.0)
        self.assertEqual(args.source_system, 255)

    def test_custom_connection(self):
        args = parse_args(['--connection', 'udpin:0.0.0.0:14550'])
        self.assertEqual(args.connection, 'udpin:0.0.0.0:14550')

    def test_landing_mode(self):
        args = parse_args(['--mode', 'landing'])
        self.assertEqual(args.mode, 'landing')

    def test_custom_rate(self):
        args = parse_args(['--rate', '10'])
        self.assertAlmostEqual(args.rate, 10.0)

    def test_source_system(self):
        args = parse_args(['--source-system', '1'])
        self.assertEqual(args.source_system, 1)

    def test_invalid_mode_raises(self):
        with self.assertRaises(SystemExit):
            parse_args(['--mode', 'invalid'])


class TestBuildPoseCovariance(unittest.TestCase):
    def test_length(self):
        cov = build_pose_covariance()
        self.assertEqual(len(cov), 21)

    def test_position_variance(self):
        cov = build_pose_covariance()
        for i in (0, 6, 11):
            self.assertAlmostEqual(cov[i], 0.05)

    def test_attitude_variance(self):
        cov = build_pose_covariance()
        for i in (15, 18, 20):
            self.assertAlmostEqual(cov[i], 0.01)

    def test_remaining_are_nan(self):
        cov = build_pose_covariance()
        known = {0, 6, 11, 15, 18, 20}
        for i, v in enumerate(cov):
            if i not in known:
                self.assertTrue(math.isnan(v), f'index {i} should be NaN, got {v}')


class TestSendVisionDemo(unittest.TestCase):
    def _make_mock(self):
        m = MagicMock()
        m.mav = MagicMock()
        return m

    def test_calls_vision_position_estimate_send(self):
        m = self._make_mock()
        cov = build_pose_covariance()
        wall_t0 = time.time()
        send_vision_demo(m, wall_t0, cov)
        m.mav.vision_position_estimate_send.assert_called_once()

    def test_usec_is_positive(self):
        m = self._make_mock()
        cov = build_pose_covariance()
        wall_t0 = time.time()
        send_vision_demo(m, wall_t0, cov)
        args = m.mav.vision_position_estimate_send.call_args[0]
        usec = args[0]
        self.assertGreater(usec, 0)

    def test_covariance_passed(self):
        m = self._make_mock()
        cov = build_pose_covariance()
        wall_t0 = time.time()
        send_vision_demo(m, wall_t0, cov)
        args = m.mav.vision_position_estimate_send.call_args[0]
        self.assertIs(args[-1], cov)


class TestSendLandingTargetDemo(unittest.TestCase):
    def test_calls_landing_target_send(self):
        m = MagicMock()
        m.mav = MagicMock()
        send_landing_target_demo(m)
        m.mav.landing_target_send.assert_called_once()

    def test_distance_positive(self):
        m = MagicMock()
        m.mav = MagicMock()
        send_landing_target_demo(m)
        args = m.mav.landing_target_send.call_args[0]
        distance = args[5]  # (usec, target_num, frame, angle_x, angle_y, distance, ...)
        self.assertGreater(distance, 0)


class TestMain(unittest.TestCase):
    def test_negative_rate_returns_1(self):
        ret = main(['--rate', '-1'])
        self.assertEqual(ret, 1)

    def test_zero_rate_returns_1(self):
        ret = main(['--rate', '0'])
        self.assertEqual(ret, 1)

    def _make_mock_connection(self, mode='vision'):
        mock_conn = MagicMock()
        mock_conn.target_system = 1
        mock_conn.target_component = 1
        mock_conn.mav = MagicMock()

        # make wait_heartbeat a no-op
        mock_conn.wait_heartbeat = MagicMock()

        # make the loop run once then raise KeyboardInterrupt
        call_count = [0]

        def side_effect(*a, **kw):
            call_count[0] += 1
            if call_count[0] >= 1:
                raise KeyboardInterrupt

        if mode == 'vision':
            mock_conn.mav.vision_position_estimate_send.side_effect = side_effect
        else:
            mock_conn.mav.landing_target_send.side_effect = side_effect

        return mock_conn

    def test_vision_mode_runs_and_returns_0(self):
        mock_conn = self._make_mock_connection('vision')
        with patch('sitl_mavlink_vision_bridge.mavutil.mavlink_connection', return_value=mock_conn):
            ret = main(['--rate', '100', '--mode', 'vision'])
        self.assertEqual(ret, 0)
        mock_conn.mav.vision_position_estimate_send.assert_called()

    def test_landing_mode_runs_and_returns_0(self):
        mock_conn = self._make_mock_connection('landing')
        with patch('sitl_mavlink_vision_bridge.mavutil.mavlink_connection', return_value=mock_conn):
            ret = main(['--rate', '100', '--mode', 'landing'])
        self.assertEqual(ret, 0)
        mock_conn.mav.landing_target_send.assert_called()


if __name__ == '__main__':
    unittest.main()
