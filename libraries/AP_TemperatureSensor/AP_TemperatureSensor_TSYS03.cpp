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

#include "AP_TemperatureSensor_TSYS03.h"

#if AP_TEMPERATURE_SENSOR_TSYS03_ENABLED
#include <utility>
#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>

#ifndef AP_TEMPERATURE_SENSOR_TSYS03_ENFORCE_KNOWN_VALID_I2C_ADDRESS
#define AP_TEMPERATURE_SENSOR_TSYS03_ENFORCE_KNOWN_VALID_I2C_ADDRESS 1
#endif

extern const AP_HAL::HAL &hal;

static const uint8_t TSYS03_CMD_RESET       = 0x1E;
static const uint8_t TSYS03_CMD_CONVERT     = 0x46;
static const uint8_t TSYS03_CMD_READ_ADC    = 0x00;

void AP_TemperatureSensor_TSYS03::init()
{
    constexpr char name[] = "TSYS03";
    (void)name;  // sometimes this is unused (e.g. HAL_GCS_ENABLED false)

#if AP_TEMPERATURE_SENSOR_TSYS03_ENFORCE_KNOWN_VALID_I2C_ADDRESS
    // I2C Address: Default to using TSYS03_ADDR_CSB0 & Check I2C Address is Correct
    if ((_params.bus_address != TSYS03_ADDR_CSB0) ) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s wrong I2C addr of 0x%2X, setting to 0x%2X", name, (unsigned)_params.bus_address.get(), (unsigned)TSYS03_ADDR_CSB0);
        _params.bus_address.set(TSYS03_ADDR_CSB0);
    }
#endif

    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s device is null!", name);
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // reset
    if (!_dev->transfer(&TSYS03_CMD_RESET, 1, nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s reset failed", name);
        return;
    }

    hal.scheduler->delay(4);

    start_next_sample();

    // lower retries for run
    _dev->set_retries(3);

    /* Request 20Hz update */
    // Max conversion time is 9.04 ms
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TSYS03::_timer, void));
}

uint16_t AP_TemperatureSensor_TSYS03::read_adc() const
{
    uint8_t val[3];
    if (!_dev->transfer(&TSYS03_CMD_READ_ADC, 1, val, 3)) {
        return 0;
    }

    // ensure crc is correct:
    uint8_t expected_crc = 0;
    for (uint8_t i=0; i<2; i++) {
        expected_crc = crc8_dvb(expected_crc, val[i], 0x31);
    }
    if (expected_crc != val[2]) {
        return 0;
    }

    return UINT16_VALUE(val[0],val[1]);
}

void AP_TemperatureSensor_TSYS03::_timer(void)
{
    const uint16_t adc = read_adc();

    if (adc != 0) {
        const float temp = calculate(adc);
        set_temperature(temp);
    }

    start_next_sample();
}

void AP_TemperatureSensor_TSYS03::start_next_sample()
{
    _dev->transfer(&TSYS03_CMD_CONVERT, 1, nullptr, 0);
}

float AP_TemperatureSensor_TSYS03::calculate(const uint16_t adc) const
{
    const float temperature = -40.0 + adc * 165 / (powf(2, 16) - 1.0);

    return temperature;
}

#endif // AP_TEMPERATURE_SENSOR_TSYS03_ENABLED
