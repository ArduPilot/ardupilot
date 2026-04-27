"""
Step response metrics calculation functions.

AP_FLAKE8_CLEAN
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from numpy.typing import ArrayLike


@dataclass
class StepResponseMetrics:
    """Results from analyzing a step response."""

    rise_time: float | None
    overshoot: float
    settling_time: float | None
    steady_state_error: float


def rise_time(time: ArrayLike, response: ArrayLike, target: float) -> float | None:
    """
    Calculate rise time (10% to 90% of target).

    Args:
        time: Time array in seconds.
        response: Response signal array.
        target: Target setpoint value.

    Returns:
        Rise time in seconds, or None if thresholds not reached.
    """
    time = np.asarray(time)
    response = np.asarray(response)

    initial = response[0]
    low = initial + 0.1 * (target - initial)
    high = initial + 0.9 * (target - initial)

    t_low = None
    t_high = None

    for i, val in enumerate(response):
        if t_low is None and val >= low:
            t_low = time[i]
        if t_high is None and val >= high:
            t_high = time[i]
            break

    if t_low is None or t_high is None:
        return None

    return t_high - t_low


def overshoot(response: ArrayLike, target: float) -> float:
    """
    Calculate percentage overshoot.

    Args:
        response: Response signal array.
        target: Target setpoint value.

    Returns:
        Overshoot as percentage (0-100+). Returns 0 if no overshoot.
    """
    response = np.asarray(response)
    initial = response[0]

    if target == initial:
        return 0.0

    peak = np.max(response) if target > initial else np.min(response)
    os = (peak - target) / (target - initial) * 100

    return max(0.0, os)


def settling_time(
    time: ArrayLike,
    response: ArrayLike,
    target: float,
    band: float = 0.05
) -> float | None:
    """
    Calculate settling time (time to stay within error band).

    Args:
        time: Time array in seconds.
        response: Response signal array.
        target: Target setpoint value.
        band: Error band as fraction of target (default 5%).

    Returns:
        Settling time in seconds, or None if never settles.
    """
    time = np.asarray(time)
    response = np.asarray(response)

    tolerance = abs(target * band)
    within_band = np.abs(response - target) <= tolerance

    outside_indices = np.where(~within_band)[0]

    if len(outside_indices) == 0:
        return 0.0

    last_outside = outside_indices[-1]

    if last_outside >= len(time) - 1:
        return None

    return time[last_outside + 1] - time[0]


def steady_state_error(
    response: ArrayLike, target: float, tail_fraction: float = 0.1
) -> float:
    """
    Calculate steady-state error (average error in final portion).

    Args:
        response: Response signal array.
        target: Target setpoint value.
        tail_fraction: Fraction of data to use from end (default 10%).

    Returns:
        Steady-state error as absolute value.
    """
    response = np.asarray(response)

    tail_size = max(1, int(len(response) * tail_fraction))
    tail = response[-tail_size:]

    return abs(np.mean(tail) - target)


def analyze_step_response(
    time: ArrayLike,
    response: ArrayLike,
    target: float,
    settling_band: float = 0.05
) -> StepResponseMetrics:
    """
    Perform complete step response analysis.

    Args:
        time: Time array in seconds.
        response: Response signal array.
        target: Target setpoint value.
        settling_band: Error band for settling time calculation.

    Returns:
        StepResponseMetrics with all computed metrics.
    """
    return StepResponseMetrics(
        rise_time=rise_time(time, response, target),
        overshoot=overshoot(response, target),
        settling_time=settling_time(time, response, target, settling_band),
        steady_state_error=steady_state_error(response, target)
    )
