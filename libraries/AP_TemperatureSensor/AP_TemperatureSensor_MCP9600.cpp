/*
 * Copyright (C) 2022  Kraus Hamdani Aerospace Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Tom Pittenger
 */
/*
    Implements I2C driver for Microchip MCP9600 digital thermocouple EMF to Temperature converter
*/

#include "AP_TemperatureSensor_MCP9600.h"

#if AP_TEMPERATURE_SENSOR_MCP9600_ENABLED

#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

static const uint8_t MCP9600_CMD_HOT_JUNCT_TEMP     = 0x00;     // thermocoupler temp
//static const uint8_t MCP9600_CMD_JUNCT_TEMP_DELTA   = 0x01;
//static const uint8_t MCP9600_CMD_COLD_JUNCT_TEMP    = 0x02;     // ambient temp
//static const uint8_t MCP9600_CMD_RAW_DATA_ADC       = 0x03;
//static const uint8_t MCP9600_CMD_STATUS             = 0x04;
static const uint8_t MCP9600_CMD_SENSOR_CFG         = 0x05;
//static const uint8_t MCP9600_CMD_DEVICE_CFG         = 0x06;
static const uint8_t MCP9600_CMD_DEVICE_ID_REV      = 0x20;     // to fetch WHOAMI


static const uint8_t MCP9600_WHOAMI                 = 0x40;


#define MCP9600_ADDR_LOW        0x60    // ADDR pin pulled low
#define MCP9600_ADDR_HIGH       0x66    // ADDR pin pulled high

#define AP_TemperatureSensor_MCP9600_UPDATE_INTERVAL_MS     500
#define AP_TemperatureSensor_MCP9600_SCALE_FACTOR           (0.0625f)

#ifndef AP_TemperatureSensor_MCP9600_ADDR_DEFAULT
    #define AP_TemperatureSensor_MCP9600_ADDR_DEFAULT    MCP9600_ADDR_LOW
#endif

#ifndef AP_TemperatureSensor_MCP9600_ENFORCE_KNOWN_VALID_I2C_ADDRESS
#define AP_TemperatureSensor_MCP9600_ENFORCE_KNOWN_VALID_I2C_ADDRESS 1
#endif

#ifndef AP_TemperatureSensor_MCP9600_Filter
    #define AP_TemperatureSensor_MCP9600_Filter         2   // can be values 0 through 7 where 0 is no filtering (fast) and 7 is lots of smoothing (very very slow)
#endif

void AP_TemperatureSensor_MCP9600::init()
{
    constexpr char name[] = "MCP9600";

#if AP_TemperatureSensor_MCP9600_ENFORCE_KNOWN_VALID_I2C_ADDRESS
    // I2C Address: Default to using MCP9600_ADDR_LOW if it's out of range
    if ((_params.bus_address < MCP9600_ADDR_LOW) || ( _params.bus_address > MCP9600_ADDR_HIGH)) {
        printf("%s wrong I2C addr of 0x%2X, setting to 0x%2X. Reboot needed.", name, (unsigned)_params.bus_address.get(), (unsigned)MCP9600_ADDR_LOW);
        _params.bus_address.set(MCP9600_ADDR_LOW);
    }
#endif

    _dev = std::move(hal.i2c_mgr->get_device(_params.bus, _params.bus_address));
    if (!_dev) {
        printf("%s device is null!", name);
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    uint8_t buf[2];
    if (!_dev->read_registers(MCP9600_CMD_DEVICE_ID_REV, buf, 2)) {
        printf("%s failed to get WHOAMI", name);
        return;
    }
    if (buf[0] != MCP9600_WHOAMI) {
        printf("%s Got wrong WHOAMI: detected 0x%2X but expected 0x%2X", name, (unsigned)buf[0], (unsigned)MCP9600_WHOAMI);
        return;
    }
    if (!set_config(ThermocoupleType::K, AP_TemperatureSensor_MCP9600_Filter)) {
        printf("%s unable to configure", name);
        return;
    }

#if 0
    printf("%s Detected! Rev %u.%u on bus: %u addr: 0x%2X", name, (unsigned)(buf[1] >> 4), (unsigned)(buf[1] & 0x0F), (unsigned)_params.bus, (unsigned)_params.bus_address);
#endif

    // lower retries for run
    _dev->set_retries(3);

    /* Request 10Hz update */
    _dev->register_periodic_callback(100 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_MCP9600::_timer, void));
}

void AP_TemperatureSensor_MCP9600::_timer()
{
    float temperature;
    if (read_temperature(temperature)) {
        set_temperature(temperature);
    }
}

bool AP_TemperatureSensor_MCP9600::set_config(const ThermocoupleType thermoType, const uint8_t filter)
{
    const uint8_t data = (uint8_t)thermoType << 4 | (filter & 0x7);
    if (!_dev->write_register(MCP9600_CMD_SENSOR_CFG, data)) {
        return false;
    }
    return true;
}

bool AP_TemperatureSensor_MCP9600::read_temperature(float &temperature)
{
    uint8_t data[2];
    if (!_dev->read_registers(MCP9600_CMD_HOT_JUNCT_TEMP, data, 2)) {
        return false;
    }

    temperature = int16_t(UINT16_VALUE(data[0], data[1])) * AP_TemperatureSensor_MCP9600_SCALE_FACTOR;
    return true;
}
#endif // AP_TEMPERATURE_SENSOR_MCP9600_ENABLED

