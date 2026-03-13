"""
Unit tests for the metrics module.

AP_FLAKE8_CLEAN
"""

import pytest
import numpy as np

from controls_lab.core.metrics import (
    rise_time,
    overshoot,
    settling_time,
    steady_state_error,
    analyze_step_response,
    StepResponseMetrics,
)


class TestRiseTime:
    """Tests for rise_time calculation."""

    def test_rise_time_ideal_response(self, ideal_step_response):
        """Rise time should be calculable for ideal step response."""
        time, response, target = ideal_step_response
        rt = rise_time(time, response, target)
        assert rt is not None
        assert rt > 0

    def test_rise_time_returns_none_if_threshold_not_reached(self):
        """Rise time should be None if 90% threshold not reached."""
        time = np.linspace(0, 1, 50)
        response = np.linspace(0, 5, 50)  # Only reaches 50% of target
        target = 10.0
        rt = rise_time(time, response, target)
        assert rt is None

    def test_rise_time_faster_response_has_smaller_value(
        self, ideal_step_response, overdamped_step_response
    ):
        """Faster response should have smaller rise time."""
        time_fast, response_fast, target_fast = ideal_step_response
        time_slow, response_slow, target_slow = overdamped_step_response

        rt_fast = rise_time(time_fast, response_fast, target_fast)
        rt_slow = rise_time(time_slow, response_slow, target_slow)

        assert rt_fast is not None
        assert rt_slow is not None
        assert rt_fast < rt_slow


class TestOvershoot:
    """Tests for overshoot calculation."""

    def test_overshoot_ideal_response_is_zero(self, ideal_step_response):
        """First-order response should have zero overshoot."""
        _, response, target = ideal_step_response
        os = overshoot(response, target)
        assert os == pytest.approx(0.0, abs=0.1)

    def test_overshoot_underdamped_is_positive(self, underdamped_step_response):
        """Underdamped response should have positive overshoot."""
        _, response, target = underdamped_step_response
        os = overshoot(response, target)
        assert os > 0

    def test_overshoot_never_negative(self):
        """Overshoot should never be negative."""
        response = np.array([0, 2, 4, 6, 8, 9, 9.5, 9.8])  # No overshoot
        target = 10.0
        os = overshoot(response, target)
        assert os >= 0


class TestSettlingTime:
    """Tests for settling_time calculation."""

    def test_settling_time_ideal_response(self, ideal_step_response):
        """Settling time should be calculable for ideal step response."""
        time, response, target = ideal_step_response
        st = settling_time(time, response, target, band=0.05)
        assert st is not None

    def test_settling_time_returns_none_if_never_settles(self):
        """Settling time should be None if response never settles."""
        time = np.linspace(0, 5, 100)
        response = np.sin(time) + 5  # Oscillating around 5, target is 10
        target = 10.0
        st = settling_time(time, response, target, band=0.05)
        assert st is None

    def test_settling_time_wider_band_is_faster(self, underdamped_step_response):
        """Wider settling band should result in faster settling time."""
        time, response, target = underdamped_step_response
        st_narrow = settling_time(time, response, target, band=0.02)
        st_wide = settling_time(time, response, target, band=0.10)

        if st_narrow is not None and st_wide is not None:
            assert st_wide <= st_narrow


class TestSteadyStateError:
    """Tests for steady_state_error calculation."""

    def test_steady_state_error_ideal_response(self, ideal_step_response):
        """Ideal response settling to target should have small SS error."""
        _, response, target = ideal_step_response
        sse = steady_state_error(response, target)
        assert sse < 1.0  # Small error expected

    def test_steady_state_error_with_offset(self, step_response_with_steady_state_error):
        """Response settling away from target should have measurable SS error."""
        _, response, target = step_response_with_steady_state_error
        sse = steady_state_error(response, target)
        assert sse > 0.4  # Expect ~0.5 error for 9.5 vs 10.0 target

    def test_steady_state_error_is_absolute(self):
        """Steady-state error should be absolute value."""
        response = np.array([0, 5, 8, 9, 9.5, 9.5, 9.5])
        target = 10.0
        sse = steady_state_error(response, target)
        assert sse >= 0


class TestAnalyzeStepResponse:
    """Tests for the aggregate analyze_step_response function."""

    def test_returns_metrics_dataclass(self, ideal_step_response):
        """Should return a StepResponseMetrics dataclass."""
        time, response, target = ideal_step_response
        metrics = analyze_step_response(time, response, target)
        assert isinstance(metrics, StepResponseMetrics)

    def test_all_fields_populated(self, ideal_step_response):
        """All metric fields should be populated."""
        time, response, target = ideal_step_response
        metrics = analyze_step_response(time, response, target)

        assert metrics.rise_time is not None
        assert metrics.overshoot is not None
        assert metrics.settling_time is not None
        assert metrics.steady_state_error is not None

    def test_custom_settling_band(self, underdamped_step_response):
        """Custom settling band parameter should be passed through."""
        time, response, target = underdamped_step_response

        metrics_narrow = analyze_step_response(time, response, target, settling_band=0.02)
        metrics_wide = analyze_step_response(time, response, target, settling_band=0.10)

        # Wide band should settle faster or equally fast
        if metrics_narrow.settling_time is not None and metrics_wide.settling_time is not None:
            assert metrics_wide.settling_time <= metrics_narrow.settling_time
