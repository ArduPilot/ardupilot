"""
Pytest fixtures for controls_lab tests.

AP_FLAKE8_CLEAN
"""

import pytest
import numpy as np


@pytest.fixture
def ideal_step_response():
    """An ideal first-order step response with no overshoot."""
    time = np.linspace(0, 5, 100)
    target = 10.0
    tau = 0.5  # time constant
    response = target * (1 - np.exp(-time / tau))
    return time, response, target


@pytest.fixture
def overdamped_step_response():
    """An overdamped step response - slow rise, no overshoot."""
    time = np.linspace(0, 10, 200)
    target = 20.0
    tau = 2.0
    response = target * (1 - np.exp(-time / tau))
    return time, response, target


@pytest.fixture
def underdamped_step_response():
    """An underdamped step response with overshoot and oscillation."""
    time = np.linspace(0, 10, 200)
    target = 15.0
    zeta = 0.3  # damping ratio
    omega_n = 2.0  # natural frequency
    omega_d = omega_n * np.sqrt(1 - zeta**2)

    response = target * (1 - np.exp(-zeta * omega_n * time) * (
        np.cos(omega_d * time) +
        (zeta / np.sqrt(1 - zeta**2)) * np.sin(omega_d * time)
    ))
    return time, response, target


@pytest.fixture
def step_response_with_steady_state_error():
    """Step response that settles to a value different from target."""
    time = np.linspace(0, 5, 100)
    target = 10.0
    actual_final = 9.5  # 5% steady-state error
    tau = 0.5
    response = actual_final * (1 - np.exp(-time / tau))
    return time, response, target
