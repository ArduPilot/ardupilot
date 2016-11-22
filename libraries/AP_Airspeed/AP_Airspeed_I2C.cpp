/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  backend driver for airspeed from a I2C MS4525D0 sensor
 */
#include "AP_Airspeed_I2C.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define MS4525D0_I2C_ADDR 0x28

#ifdef HAL_AIRSPEED_MS4515DO_I2C_BUS
#define MS4525D0_I2C_BUS HAL_AIRSPEED_MS4515DO_I2C_BUS
#else
#define MS4525D0_I2C_BUS 1
#endif

AP_Airspeed_I2C::AP_Airspeed_I2C(const AP_Float &psi_range) :
    _psi_range(psi_range)
{
}

// probe and initialise the sensor
bool AP_Airspeed_I2C::init()
{
    _dev = hal.i2c_mgr->get_device(MS4525D0_I2C_BUS, MS4525D0_I2C_ADDR);

    // take i2c bus sempahore
    if (!_dev || !_dev->get_semaphore()->take(200)) {
        return false;
    }

    // lots of retries during probe
    _dev->set_retries(5);
    
    //_measure();
    //hal.scheduler->delay(10);
    _collect();
    _dev->get_semaphore()->give();

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    if (_last_sample_time_ms != 0) {
        _dev->register_periodic_callback(20000,
                                         FUNCTOR_BIND_MEMBER(&AP_Airspeed_I2C::_timer, bool));
        return true;
    }
    return false;
}

// start a measurement
void AP_Airspeed_I2C::_measure()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0;
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

// read the values from the sensor
void AP_Airspeed_I2C::_collect()
{
    uint8_t val[4];

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, val, sizeof(val))) {
        return;
    }

    status = (val[0] & 0xc0) >> 6;  // first 2 bits from first byte
    bridge_data = ((val[0] & 0x3f) << 8) + val[1];
    temperature_data = ((val[2] << 8) + (val[3] & 0xe0)) >> 5;
    if ( temperature_data == 65535 ) return;

    float output_min = 1638;
    float output_max = 14745;
    float pressure_min = -249.0889;
    float pressure_max = 249.0889;

    _pressure = 1.0 * (bridge_data - output_min) * (pressure_max - pressure_min) / (output_max - output_min) + pressure_min;
    _temperature = (temperature_data * 0.0977) - 50;

    //_voltage_correction(_pressure, _temperature);
    
    _last_sample_time_ms = AP_HAL::millis();
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void AP_Airspeed_I2C::_voltage_correction(float &diff_press_pa, float &temperature)
{
	const float slope = 65.0f;
	const float temp_slope = 0.887f;

	/*
	  apply a piecewise linear correction within range given by above graph
	 */
	float voltage_diff = hal.analogin->board_voltage() - 5.0f;

    voltage_diff = constrain_float(voltage_diff, -0.7f, 0.5f);

	diff_press_pa -= voltage_diff * slope;
	temperature -= voltage_diff * temp_slope;
}

// 50Hz timer
bool AP_Airspeed_I2C::_timer()
{
    if (_measurement_started_ms == 0) {
        _measure();
        return true;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
    return true;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_I2C::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_I2C::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    temperature = _temperature;
    return true;
}
