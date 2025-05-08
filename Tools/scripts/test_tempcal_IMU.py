#!/usr/bin/python3

"""
Tests for the tempcal_IMU.py file.

SPDX-FileCopyrightText: 2024-2025 Amilcar do Carmo Lucas <amilcar.lucas@iav.de>

SPDX-License-Identifier: GPL-3.0-or-later
"""

import os
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import pytest
from pymavlink.rotmat import Vector3

from .tempcal_IMU import (
    AXES,
    POLY_ORDER,
    Coefficients,
    IMUData,
    constrain,
    generate_calibration_file,
    generate_tempcal_accel_figures,
    generate_tempcal_gyro_figures,
)


class TestCoefficients(unittest.TestCase):
    """Test the Coefficients class."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        self.c = Coefficients()

    def test_initial_values(self) -> None:
        """Test initial values are correctly set."""
        assert self.c.enable == [0, 0, 0]
        assert self.c.tmin == [-100, -100, -100]
        assert self.c.tmax == [-100, -100, -100]
        assert not self.c.acoef
        assert not self.c.gcoef

    def test_set_accel_poly(self) -> None:
        """Test setting accel polynomial coefficients."""
        # When setting values for first time
        self.c.set_accel_poly(0, "X", [1.0, 0.1, 0.01, 0.001])
        assert 0 in self.c.acoef
        assert "X" in self.c.acoef[0]
        assert self.c.acoef[0]["X"] == [1.0, 0.1, 0.01, 0.001]

        # Update existing values
        self.c.set_accel_poly(0, "X", [2.0, 0.2, 0.02, 0.002])
        assert self.c.acoef[0]["X"] == [2.0, 0.2, 0.02, 0.002]

        values = [1.0, 2.0, 3.0, 4.0]
        self.c.set_accel_poly(0, "X", values)
        assert self.c.acoef[0]["X"] == values

        # Test adding another axis
        values2 = [5.0, 6.0, 7.0, 8.0]
        self.c.set_accel_poly(0, "Y", values2)
        assert self.c.acoef[0]["Y"] == values2

        # Test adding another IMU
        values3 = [9.0, 10.0, 11.0, 12.0]
        self.c.set_accel_poly(1, "X", values3)
        assert self.c.acoef[1]["X"] == values3

    def test_set_gyro_poly(self) -> None:
        """Test setting gyro polynomial coefficients."""
        values = [1.0, 2.0, 3.0, 4.0]
        self.c.set_gyro_poly(0, "X", values)
        assert self.c.gcoef[0]["X"] == values

        self.c.set_gyro_poly(0, "X", [0.1, 0.01, 0.001, 0.0001])
        assert 0 in self.c.gcoef
        assert "X" in self.c.gcoef[0]
        assert self.c.gcoef[0]["X"] == [0.1, 0.01, 0.001, 0.0001]

    def test_set_tmin_tmax(self) -> None:
        """Test setting temperature min and max values."""
        self.c.set_tmin(0, 20.0)
        self.c.set_tmax(0, 40.0)
        assert self.c.tmin[0] == 20.0
        assert self.c.tmax[0] == 40.0

    def test_correction(self) -> None:
        """Test the correction calculation."""
        # Setup
        self.c.set_enable(0, 1)
        self.c.set_tmin(0, 20.0)
        self.c.set_tmax(0, 40.0)
        # Using different polynomial coefficients that will definitely produce non-zero results
        self.c.set_accel_poly(0, "X", [0.0, 0.0, 1.0, 0.0])  # This creates a simple linear function

        # Test correction with temperature in range
        coeff = self.c.acoef[0]
        correction = self.c.correction(coeff, 0, 30.0, "X", 35.0)
        assert correction != 0.0

        # Test with disabled IMU
        self.c.set_enable(0, 0)
        correction = self.c.correction(coeff, 0, 30.0, "X", 35.0)
        assert correction == 0.0

        # Test with out of range cal_temp
        self.c.set_enable(0, 1)
        correction = self.c.correction(coeff, 0, 30.0, "X", -90.0)
        assert correction == 0.0

        # Test with missing axis
        correction = self.c.correction(coeff, 0, 30.0, "Z", 35.0)
        assert correction == 0.0

        # Re-enable and test with out of range temperature
        self.c.set_enable(0, 1)
        # Test with temperature out of range (should be constrained)
        correction_constrained = self.c.correction(coeff, 0, 15.0, "X", 35.0)
        assert correction_constrained != 0.0

    def test_param_string(self) -> None:
        """Test generating parameter string."""
        # Setup
        self.c.set_tmin(0, 20.0)
        self.c.set_tmax(0, 40.0)
        self.c.set_accel_poly(0, "X", [1.0, 0.1, 0.01, 0.001])
        self.c.set_accel_poly(0, "Y", [2.0, 0.2, 0.02, 0.002])
        self.c.set_accel_poly(0, "Z", [3.0, 0.3, 0.03, 0.003])
        self.c.set_gyro_poly(0, "X", [4.0, 0.4, 0.04, 0.004])
        self.c.set_gyro_poly(0, "Y", [5.0, 0.5, 0.05, 0.005])
        self.c.set_gyro_poly(0, "Z", [6.0, 0.6, 0.06, 0.006])

        # Test
        param_str = self.c.param_string(0)

        # Verify
        assert "INS_TCAL1_ENABLE 1" in param_str
        assert "INS_TCAL1_TMIN 20.0" in param_str
        assert "INS_TCAL1_TMAX 40.0" in param_str
        for p in range(POLY_ORDER):
            for axis in AXES:
                assert f"INS_TCAL1_ACC{p + 1}_{axis}" in param_str
                assert f"INS_TCAL1_GYR{p + 1}_{axis}" in param_str


class TestIMUData(unittest.TestCase):
    """Test the IMUData class."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        self.data = IMUData()

    def test_add_accel(self) -> None:
        """Test adding acceleration data."""
        self.data.add_accel(0, 25.0, 1.0, Vector3(1.0, 2.0, 3.0))
        self.data.add_accel(0, 26.0, 2.0, Vector3(1.1, 2.1, 3.1))

        # Verify data was added correctly
        assert len(self.data.accel[0]["T"]) == 2
        assert len(self.data.accel[0]["X"]) == 2
        assert len(self.data.accel[0]["time"]) == 2
        assert self.data.accel[0]["T"][0] == 25.0
        assert self.data.accel[0]["X"][1] == 1.1
        assert self.data.accel[0]["time"][1] == 2.0

    def test_add_gyro(self) -> None:
        """Test adding gyro data."""
        self.data.add_gyro(0, 25.0, 1.0, Vector3(0.1, 0.2, 0.3))
        self.data.add_gyro(0, 26.0, 2.0, Vector3(0.11, 0.21, 0.31))

        # Verify data was added correctly
        assert len(self.data.gyro[0]["T"]) == 2
        assert len(self.data.gyro[0]["X"]) == 2
        assert self.data.gyro[0]["Y"][1] == 0.21

    @patch("sys.exit")
    def test_imus(self, mock_exit) -> None:
        """Test getting IMU list."""
        # Add data for IMU 0
        self.data.add_accel(0, 25.0, 1.0, Vector3(1.0, 2.0, 3.0))
        self.data.add_gyro(0, 25.0, 1.0, Vector3(0.1, 0.2, 0.3))

        # Add data for IMU 1
        self.data.add_accel(1, 25.0, 1.0, Vector3(1.0, 2.0, 3.0))
        self.data.add_gyro(1, 25.0, 1.0, Vector3(0.1, 0.2, 0.3))

        # Test getting IMU list
        imus = self.data.IMUs()
        assert len(imus) == 2
        assert 0 in imus
        assert 1 in imus

        # Test mismatched data
        del self.data.gyro[1]
        imus = self.data.IMUs()
        mock_exit.assert_called_once()

    def test_moving_average(self) -> None:
        """Test moving average filter."""
        data = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        result = self.data.moving_average(data, 3)
        # Expected: [2.0, 3.0, 4.0]
        np.testing.assert_array_equal(result, np.array([2.0, 3.0, 4.0]))

    def test_accel_at_temp(self) -> None:
        """Test getting acceleration at a specific temperature."""
        # Add data points with increasing temperatures
        self.data.add_accel(0, 25.0, 1.0, Vector3(1.0, 2.0, 3.0))
        self.data.add_accel(0, 30.0, 2.0, Vector3(1.5, 2.5, 3.5))
        self.data.add_accel(0, 35.0, 3.0, Vector3(2.0, 3.0, 4.0))

        # Test exact temperature match
        value = self.data.accel_at_temp(0, "X", 30.0)
        assert value == 1.5

        # Test interpolation between temperatures
        value = self.data.accel_at_temp(0, "X", 27.5)
        assert value == 1.25  # Linear interpolation between 1.0 and 1.5

        # Test below minimum temperature
        value = self.data.accel_at_temp(0, "X", 20.0)
        assert value == 1.0  # Should return first value

        # Test above maximum temperature
        value = self.data.accel_at_temp(0, "X", 40.0)
        assert value == 2.0  # Should return last value


class TestUtilityFunctions(unittest.TestCase):
    """Test utility functions."""

    def test_constrain(self) -> None:
        """Test the constrain function."""
        # Test within bounds
        assert constrain(5.0, 0.0, 10.0) == 5.0

        # Test below minimum
        assert constrain(-5.0, 0.0, 10.0) == 0.0

        # Test above maximum
        assert constrain(15.0, 0.0, 10.0) == 10.0


@pytest.fixture
def temp_output_file() -> str:
    """Fixture to create a temporary output file."""
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        yield tmp.name
    # Clean up after test
    if os.path.exists(tmp.name):
        os.unlink(tmp.name)


def test_generate_calibration_file(temp_output_file) -> None:  # pylint: disable=redefined-outer-name
    """Test generating calibration file."""
    # Create mock IMUData
    data = IMUData()

    # Add some test data
    for t in range(20, 41):  # Temperature range 20-40
        data.add_accel(0, t, t - 19, Vector3(t * 0.1, t * 0.2, t * 0.3))
        data.add_gyro(0, t, t - 19, Vector3(t * 0.01, t * 0.02, t * 0.03))

    # Create initial coefficients
    c = Coefficients()

    # Call function
    _new_c, _clog = generate_calibration_file(temp_output_file, None, data, c, online=False)

    # Check file was created
    assert os.path.exists(temp_output_file)

    # Check file content
    with open(temp_output_file, encoding="utf-8") as f:
        content = f.read()
        # Check basic params are present
        assert "INS_TCAL1_ENABLE 1" in content
        assert "INS_TCAL1_TMIN 20.0" in content
        assert "INS_TCAL1_TMAX 40.0" in content
        # Check coefficients are present
        for p in range(POLY_ORDER):
            for axis in AXES:
                assert f"INS_TCAL1_ACC{p + 1}_{axis}" in content
                assert f"INS_TCAL1_GYR{p + 1}_{axis}" in content


@patch("matplotlib.pyplot.show")
def test_generate_tempcal_figures_without_figpath(mock_show) -> None:  # noqa: ARG001 # pylint: disable=unused-argument
    """Test generating temperature calibration figures without figpath parameter."""
    # Create test data
    data = IMUData()
    for t in range(20, 41):
        data.add_accel(0, t, t - 19, Vector3(t * 0.1, t * 0.2, t * 0.3))
        data.add_gyro(0, t, t - 19, Vector3(t * 0.01, t * 0.02, t * 0.03))

    # Create coefficients
    c = Coefficients()
    c.set_enable(0, 1)
    c.set_tmin(0, 20.0)
    c.set_tmax(0, 40.0)

    # Add polynomial coefficients
    for axis in AXES:
        c.set_accel_poly(0, axis, [0.001, 0.01, 0.1, 1.0])
        c.set_gyro_poly(0, axis, [0.0001, 0.001, 0.01, 0.1])

    # Test generating figures without figpath (should not call savefig)
    with patch("matplotlib.pyplot.savefig") as mock_savefig:
        generate_tempcal_gyro_figures(None, data, c, c, 1, log_parm=False)
        generate_tempcal_accel_figures(None, data, c, c, 1, log_parm=False)

        # Check savefig was not called
        assert mock_savefig.call_count == 0


def test_generate_tempcal_figures_with_log_parm() -> None:
    """Test generating temperature calibration figures with log_parm=True."""
    # Create test data
    data = IMUData()
    for t in range(20, 41):
        data.add_accel(0, t, t - 19, Vector3(t * 0.1, t * 0.2, t * 0.3))
        data.add_gyro(0, t, t - 19, Vector3(t * 0.01, t * 0.02, t * 0.03))

    # Create coefficients
    c = Coefficients()
    c.set_enable(0, 1)
    c.set_tmin(0, 20.0)
    c.set_tmax(0, 40.0)

    # Add polynomial coefficients
    for axis in AXES:
        c.set_accel_poly(0, axis, [0.001, 0.01, 0.1, 1.0])
        c.set_gyro_poly(0, axis, [0.0001, 0.001, 0.01, 0.1])

    # Create log coefficients with different values
    clog = Coefficients()
    clog.set_enable(0, 1)
    clog.set_tmin(0, 20.0)
    clog.set_tmax(0, 40.0)
    clog.set_gyro_tcal(0, 30.0)
    clog.set_accel_tcal(0, 30.0)

    # Add different polynomial coefficients for log
    for axis in AXES:
        clog.set_accel_poly(0, axis, [0.002, 0.02, 0.2, 2.0])
        clog.set_gyro_poly(0, axis, [0.0002, 0.002, 0.02, 0.2])
        clog.set_aoffset(0, axis, 1.0)
        clog.set_goffset(0, axis, 0.1)

    # Test generating figures with log_parm=True
    with patch("matplotlib.pyplot.savefig"), patch("matplotlib.pyplot.show"):
        generate_tempcal_gyro_figures(None, data, c, clog, 1, log_parm=True)
        generate_tempcal_accel_figures(None, data, c, clog, 1, log_parm=True)
        # No assertions needed - just verifying it doesn't crash


def test_generate_tempcal_figures_multiple_imus() -> None:
    """Test generating temperature calibration figures with multiple IMUs."""
    # Create test data for two IMUs
    data = IMUData()
    for imu in [0, 1]:
        for t in range(20, 41):
            data.add_accel(imu, t, t - 19, Vector3(t * 0.1, t * 0.2, t * 0.3))
            data.add_gyro(imu, t, t - 19, Vector3(t * 0.01, t * 0.02, t * 0.03))

    # Create coefficients for both IMUs
    c = Coefficients()
    for imu in [0, 1]:
        c.set_enable(imu, 1)
        c.set_tmin(imu, 20.0)
        c.set_tmax(imu, 40.0)

        # Add polynomial coefficients
        for axis in AXES:
            c.set_accel_poly(imu, axis, [0.001, 0.01, 0.1, 1.0])
            c.set_gyro_poly(imu, axis, [0.0001, 0.001, 0.01, 0.1])

    # Test generating figures with multiple IMUs
    with patch("matplotlib.pyplot.savefig"), patch("matplotlib.pyplot.show"):
        generate_tempcal_gyro_figures(None, data, c, c, 2, log_parm=False)
        generate_tempcal_accel_figures(None, data, c, c, 2, log_parm=False)
        # No assertions needed - just verifying it doesn't crash


if __name__ == "__main__":
    unittest.main()
