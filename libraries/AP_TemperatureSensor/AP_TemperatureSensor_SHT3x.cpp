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

Written with reference to the PX4 driver written by Roman Dvorak <dvorakroman@thunderfly.cz>

*/

#include "AP_TemperatureSensor_SHT3x.h"

#if AP_TEMPERATURE_SENSOR_SHT3X_ENABLED
#include <utility>
#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

void AP_TemperatureSensor_SHT3x::init()
{
    constexpr char name[] = "SHT3x";
    (void)name;  // sometimes this is unused (e.g. HAL_GCS_ENABLED false)

    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s device is null!", name);
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // read serial number
    static const uint8_t read_sn_cmd[2] { 0x37, 0x80 };
    uint8_t sn[6];
    if (!_dev->transfer(read_sn_cmd, ARRAY_SIZE(read_sn_cmd), sn, ARRAY_SIZE(sn))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s read sn failed", name);
        return;
    }
    // emit serial number, more of confirmation we have the sensor:
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SHT3x: SN%x%x%x%x%x%x", sn[0], sn[1], sn[2], sn[3], sn[4], sn[5]);

    // reset
    static const uint8_t soft_reset_cmd[2] { 0x30, 0xA2 };  // page 12
    if (!_dev->transfer(soft_reset_cmd, ARRAY_SIZE(soft_reset_cmd), nullptr, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s reset failed", name);
        return;
    }

    hal.scheduler->delay(4);

    start_next_sample();

    // lower retries for run
    _dev->set_retries(3);

    /* Request 20Hz update */
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_SHT3x::_timer, void));
}

bool AP_TemperatureSensor_SHT3x::read_measurements(uint16_t &temp, uint16_t &humidity) const
{
    uint8_t val[6];
    if (!_dev->transfer(nullptr, 1, val, ARRAY_SIZE(val))) {
        return 0;
    }

    if (val[2] != crc8_generic(&val[0], 2, 0x31, 0xff)) {
        // temperature CRC is incorrect
        return false;
    }

    if (val[5] != crc8_generic(&val[3], 2, 0x31, 0xff)) {
        // humidity CRC is incorrect
        return false;
    }

    temp = val[0] << 8 | val[1];
    humidity = val[3] << 8 | val[4];
    return true;
}

void AP_TemperatureSensor_SHT3x::_timer(void)
{
    uint16_t encoded_temp;
    uint16_t encoded_humidity;
    if (read_measurements(encoded_temp, encoded_humidity)) {
        const float temp = -45 + 175 * (encoded_temp/65535.0);
        set_temperature(temp);
    }

    start_next_sample();
}

void AP_TemperatureSensor_SHT3x::start_next_sample()
{
    static const uint8_t start_measurement_command[2] { 0x2c, 0x06 };
    _dev->transfer(start_measurement_command, ARRAY_SIZE(start_measurement_command), nullptr, 0);
}

#endif // AP_TEMPERATURE_SENSOR_SHT3X_ENABLED
