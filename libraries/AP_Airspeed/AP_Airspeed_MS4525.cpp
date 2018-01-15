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
#include "AP_Airspeed_MS4525.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define MS4525D0_I2C_ADDR 0x28

AP_Airspeed_MS4525::AP_Airspeed_MS4525(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

// probe and initialise the sensor
bool AP_Airspeed_MS4525::init()
{
    const struct {
        uint8_t bus;
        uint8_t addr;
    } addresses[] = {
        { 1, MS4525D0_I2C_ADDR },
        { 0, MS4525D0_I2C_ADDR },
        { 2, MS4525D0_I2C_ADDR },
    };
    bool found = false;
    for (uint8_t i=0; i<ARRAY_SIZE(addresses); i++) {
        _dev = hal.i2c_mgr->get_device(addresses[i].bus, addresses[i].addr);
        if (!_dev) {
            continue;
        }
        if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            continue;
        }

        // lots of retries during probe
        _dev->set_retries(10);
    
        _measure();
        hal.scheduler->delay(10);
        _collect();

        _dev->get_semaphore()->give();

        found = (_last_sample_time_ms != 0);
        if (found) {
            printf("MS4525: Found sensor on bus %u address 0x%02x\n", addresses[i].bus, addresses[i].addr);
            break;
        }
    }
    if (!found) {
        printf("MS4525: no sensor found\n");
        return false;
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_MS4525::_timer, void));
    return true;
}

// start a measurement
void AP_Airspeed_MS4525::_measure()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0;
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

/*
  this equation is an inversion of the equation in the
  pressure transfer function figure on page 4 of the datasheet
  
  We negate the result so that positive differential pressures
  are generated when the bottom port is used as the static
  port on the pitot and top port is used as the dynamic port
*/
float AP_Airspeed_MS4525::_get_pressure(int16_t dp_raw) const
{
    const float P_max = get_psi_range();
    const float P_min = - P_max;
    const float PSI_to_Pa = 6894.757f;

    float diff_press_PSI  = -((dp_raw - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
    float press  = diff_press_PSI * PSI_to_Pa;
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Airspeed_MS4525::_get_temperature(int16_t dT_raw) const
{
    float temp  = ((200.0f * dT_raw) / 2047) - 50;
    return temp;
}

// read the values from the sensor
void AP_Airspeed_MS4525::_collect()
{
    uint8_t data[4];
    uint8_t data2[4];

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return;
    }
    // reread the data, so we can attempt to detect bad inputs
    if (!_dev->transfer(nullptr, 0, data2, sizeof(data2))) {
        return;
    }

    uint8_t status = (data[0] & 0xC0) >> 6;
    // only check the status on the first read, the second read is expected to be stale
    if (status == 2 || status == 3) {
        return;
    }

    int16_t dp_raw, dT_raw;
    dp_raw = (data[0] << 8) + data[1];
    dp_raw = 0x3FFF & dp_raw;
    dT_raw = (data[2] << 8) + data[3];
    dT_raw = (0xFFE0 & dT_raw) >> 5;

    int16_t dp_raw2, dT_raw2;
    dp_raw2 = (data2[0] << 8) + data2[1];
    dp_raw2 = 0x3FFF & dp_raw2;
    dT_raw2 = (data2[2] << 8) + data2[3];
    dT_raw2 = (0xFFE0 & dT_raw2) >> 5;

    // reject any values that are the absolute minimum or maximums these
    // can happen due to gnd lifts or communication errors on the bus
    if (dp_raw  == 0x3FFF || dp_raw  == 0 || dT_raw  == 0x7FF || dT_raw == 0 ||
        dp_raw2 == 0x3FFF || dp_raw2 == 0 || dT_raw2 == 0x7FF || dT_raw2 == 0) {
        return;
    }

    // reject any double reads where the value has shifted in the upper more than
    // 0xFF
    if (abs(dp_raw - dp_raw2) > 0xFF || abs(dT_raw - dT_raw2) > 0xFF) {
        return;
    }

    float press  = _get_pressure(dp_raw);
    float press2 = _get_pressure(dp_raw2);
    float temp  = _get_temperature(dT_raw);
    float temp2 = _get_temperature(dT_raw2);
    
    _voltage_correction(press, temp);
    _voltage_correction(press2, temp2);

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _press_sum += press + press2;
        _temp_sum += temp + temp2;
        _press_count += 2;
        _temp_count += 2;
        sem->give();
    }
    
    _last_sample_time_ms = AP_HAL::millis();
}

/**
   correct for 5V rail voltage if the system_power ORB topic is
   available

   See http://uav.tridgell.net/MS4525/MS4525-offset.png for a graph of
   offset versus voltage for 3 sensors
 */
void AP_Airspeed_MS4525::_voltage_correction(float &diff_press_pa, float &temperature)
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
void AP_Airspeed_MS4525::_timer()
{
    if (_measurement_started_ms == 0) {
        _measure();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_MS4525::get_differential_pressure(float &pressure)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_press_count > 0) {
            _pressure = _press_sum / _press_count;
            _press_count = 0;
            _press_sum = 0;
        }
        sem->give();
    }
    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_MS4525::get_temperature(float &temperature)
{
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }
    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (_temp_count > 0) {
            _temperature = _temp_sum / _temp_count;
            _temp_count = 0;
            _temp_sum = 0;
        }
        sem->give();
    }
    temperature = _temperature;
    return true;
}
