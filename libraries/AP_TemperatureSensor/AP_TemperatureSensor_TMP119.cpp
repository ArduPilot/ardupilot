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

#include "AP_TemperatureSensor_TMP119.h"

#if AP_TEMPERATURE_SENSOR_TMP119_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// register addresses
#define TMP119_REG_TEMP         0x00    // temperature result register
#define TMP119_REG_CONFIG       0x01    // configuration register
#define TMP119_REG_DEVICE_ID    0x0F    // device ID register

#define TMP119_DEVICE_ID        0x2117  // value reported in the device ID register

// each LSB of the temperature register is 7.8125 m degrees C
#define TMP119_TEMP_PER_LSB     0.0078125f

// configuration register fields
#define TMP119_CONFIG_MOD_CONTINUOUS    (0x0 << 10) // continuous conversion mode
#define TMP119_CONFIG_CONV_15MS         (0x0 << 7)  // 15.5ms conversion cycle
#define TMP119_CONFIG_AVG_NONE          (0x0 << 5)  // no averaging

// continuous conversion, no averaging, shortest (15.5ms / ~64Hz) conversion
// cycle so that a fresh sample is available for every 20Hz poll
#define TMP119_CONFIG_VALUE     (TMP119_CONFIG_MOD_CONTINUOUS | \
                                 TMP119_CONFIG_CONV_15MS | \
                                 TMP119_CONFIG_AVG_NONE)

void AP_TemperatureSensor_TMP119::init()
{
    _params.bus_address.set_default(AP_TEMPERATURE_SENSOR_TMP119_DEFAULT_I2C_ADDR);

    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // confirm we are talking to a TMP119
    uint16_t device_id;
    if (!read_registers(TMP119_REG_DEVICE_ID, device_id) || (device_id != TMP119_DEVICE_ID)) {
        return;
    }

    // configure for continuous conversion with no averaging and the shortest
    // conversion cycle so a new measurement is ready for each 20Hz poll. The
    // power-up default is a 1Hz cycle with 8x averaging, which is too slow.
    if (!write_register(TMP119_REG_CONFIG, TMP119_CONFIG_VALUE)) {
        return;
    }

    // lower retries for run
    _dev->set_retries(3);

    // poll the temperature register at 20Hz
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TMP119::_timer, void));
}

void AP_TemperatureSensor_TMP119::_timer(void)
{
    uint16_t raw;
    if (!read_registers(TMP119_REG_TEMP, raw)) {
        return;
    }

    // the temperature register holds a signed 16-bit value
    const float temp = (int16_t)raw * TMP119_TEMP_PER_LSB;
    set_temperature(temp);
}

bool AP_TemperatureSensor_TMP119::read_registers(uint8_t reg, uint16_t &value) const
{
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, sizeof(val))) {
        return false;
    }
    value = UINT16_VALUE(val[0], val[1]);
    return true;
}

bool AP_TemperatureSensor_TMP119::write_register(uint8_t reg, uint16_t value) const
{
    // registers are 16-bit big-endian
    uint8_t buf[3] { reg, uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return _dev->transfer(buf, sizeof(buf), nullptr, 0);
}

#endif // AP_TEMPERATURE_SENSOR_TMP119_ENABLED
