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

#include "AP_TemperatureSensor_TSYS01.h"

#if AP_TEMPERATURE_SENSOR_TSYS01_ENABLED
#include <utility>
#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#ifndef AP_TEMPERATURE_SENSOR_TSYS01_ENFORCE_KNOWN_VALID_I2C_ADDRESS
#define AP_TEMPERATURE_SENSOR_TSYS01_ENFORCE_KNOWN_VALID_I2C_ADDRESS 1
#endif

extern const AP_HAL::HAL &hal;

static const uint8_t TSYS01_CMD_RESET       = 0x1E;
static const uint8_t TSYS01_CMD_READ_PROM   = 0xA0;
static const uint8_t TSYS01_CMD_CONVERT     = 0x40;
static const uint8_t TSYS01_CMD_READ_ADC    = 0x00;

void AP_TemperatureSensor_TSYS01::init()
{
    constexpr char name[] = "TSYS01";

#if AP_TEMPERATURE_SENSOR_TSYS01_ENFORCE_KNOWN_VALID_I2C_ADDRESS
    // I2C Address: Default to using TSYS01_ADDR_CSB0 & Check I2C Address is Correct
    if ((_params.bus_address != TSYS01_ADDR_CSB0) && (_params.bus_address != TSYS01_ADDR_CSB1)) {
        printf("%s wrong I2C addr of 0x%2X, setting to 0x%2X", name, (unsigned)_params.bus_address.get(), (unsigned)TSYS01_ADDR_CSB0);
        _params.bus_address.set(TSYS01_ADDR_CSB0);
    }
#endif

    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        printf("%s device is null!", name);
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // reset
    if (!_dev->transfer(&TSYS01_CMD_RESET, 1, nullptr, 0)) {
        printf("%s reset failed", name);
        return;
    }

    hal.scheduler->delay(4);

    if (!read_prom()) {
        printf("%s prom read failed", name);
        return;
    }

    start_next_sample();

    // lower retries for run
    _dev->set_retries(3);

    /* Request 20Hz update */
    // Max conversion time is 9.04 ms
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TSYS01::_timer, void));
}

// Register map
// prom word	Address
//      0       0xA0 -> unused
//      1       0xA2 -> _k[4]
//      2       0xA4 -> _k[3]
//      3       0xA6 -> _k[2]
//      4       0xA8 -> _k[1]
//      5       0xAA -> _k[0]
//      6       0xAC -> unused
//      7       0xAE -> unused
bool AP_TemperatureSensor_TSYS01::read_prom()
{
    bool success = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(_k); i++) {
        // Read only the prom values that we use
        _k[i] = read_prom_word(ARRAY_SIZE(_k)-i);
        success |= (_k[i] != 0);
    }
    return success;
}

// Borrowed from MS Baro driver
uint16_t AP_TemperatureSensor_TSYS01::read_prom_word(const uint8_t word) const
{
    const uint8_t reg = TSYS01_CMD_READ_PROM + (word << 1);
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, 2)) {
        return 0;
    }
    return UINT16_VALUE(val[0], val[1]);
}

uint32_t AP_TemperatureSensor_TSYS01::read_adc() const
{
    uint8_t val[3];
    if (!_dev->transfer(&TSYS01_CMD_READ_ADC, 1, val, 3)) {
        return 0;
    }
    return UINT32_VALUE(0,val[0],val[1],val[2]);
}

void AP_TemperatureSensor_TSYS01::_timer(void)
{
    const uint32_t adc = read_adc();

    if (adc != 0) {
        const float temp = calculate(adc);
        set_temperature(temp);
    }

    start_next_sample();
}

void AP_TemperatureSensor_TSYS01::start_next_sample()
{
    _dev->transfer(&TSYS01_CMD_CONVERT, 1, nullptr, 0);
}

float AP_TemperatureSensor_TSYS01::calculate(const uint32_t adc) const
{
    const float adc16 = adc/256;
    const float temperature =
        -2   * _k[4] * powf(10, -21) * powf(adc16, 4) +
        4    * _k[3] * powf(10, -16) * powf(adc16, 3) +
        -2   * _k[2] * powf(10, -11) * powf(adc16, 2) +
        1    * _k[1] * powf(10, -6)  * adc16 +
        -1.5 * _k[0] * powf(10, -2);

    return temperature;
}

#endif // AP_TEMPERATURE_SENSOR_TSYS01_ENABLED
