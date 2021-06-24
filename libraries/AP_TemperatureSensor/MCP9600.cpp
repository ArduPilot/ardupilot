/*
 * Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.
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

#include "MCP9600.h"

#if HAL_TEMP_SENSOR_MCP9600_ENABLE

#include <stdio.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint8_t MCP9600_CMD_HOT_JUNCT_TEMP     = 0x00;     // thermocoupler temp
static const uint8_t MCP9600_CMD_JUNCT_TEMP_DELTA   = 0x01;
static const uint8_t MCP9600_CMD_COLD_JUNCT_TEMP    = 0x02;     // ambient temp
static const uint8_t MCP9600_CMD_RAW_DATA_ADC       = 0x03;
static const uint8_t MCP9600_CMD_STATUS             = 0x04;
static const uint8_t MCP9600_CMD_SENSOR_CFG         = 0x05;
static const uint8_t MCP9600_CMD_DEVICE_CFG         = 0x06;
static const uint8_t MCP9600_CMD_DEVICE_ID_REV      = 0x20;     // to fetch WHOAMI


static const uint8_t MCP9600_WHOAMI                 = 0x40;


#define MCP9600_ADDR_LOW        0x60    // ADDR pin pulled low
#define MCP9600_ADDR_HIGH       0x66    // ADDR pin pulled high

#ifndef MCP9600_ADDR_DEFAULT
    #define MCP9600_ADDR_DEFAULT    MCP9600_ADDR_LOW
#endif

// table of user-configurable parameters
const AP_Param::GroupInfo MCP9600::var_info[] = {

    // @Param: BUS
    // @DisplayName: Temperature Sensor I2C bus number
    // @Description: Temperature Sensor I2C bus number
    // @Range: 0 3
    // @User: Standard
    AP_GROUPINFO("BUS", 1, MCP9600, _bus, 0),

    // @Param: ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the I2C and UAVCAN sensors to allow for multiple sensors on different addresses.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 2, MCP9600, _address, MCP9600_ADDR_DEFAULT),

    // @Param: TYPE
    // @DisplayName: Thermocoupler Type
    // @Description: Thermocoupler Type where Type-K is the most common
    // @User: Advanced
    // @Values: 0:K, 1:J, 2:T, 3:N, 4:S, 5:E, 6:B, 7:R
    AP_GROUPINFO("TYPE", 3, MCP9600, _thermoType_param, (int8_t)ThermocoupleType::K),

    // @Param: FILTER
    // @DisplayName: Sensor LP Filter
    // @Description: Sensor LP Filter. Higher values are smoother/slower temperatures changes. 0 is fastest (no filter), 7 is very very smooth/slow.
    // @Range: 0 7
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FILTER", 4, MCP9600, _filter_param, 1),

    // @Param: ID
    // @DisplayName: Sensor ID
    // @Description: Sensor ID is used to tie this temp sensor to an ID source. Example: if TEMP_ID_SRC=0 (ESC) then set this to match the ESC index. For TEMP_ID_SRC=1 (battery) set BATT_SERIAL_NUM you want and that battery will use this temp sensor
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ID", 5, MCP9600, _id, 0),

    // @Param: ID_SRC
    // @DisplayName: Sensor ID Source
    // @Description: Sensor ID Source is used to populate the temperature to particular systems so you know what the temperature is from
    // @Values: 0:ESC, 1:Battery
    // @User: Standard
    AP_GROUPINFO("ID_SRC", 6, MCP9600, _id_src, (int8_t)ID_Src::ESC),

    AP_GROUPEND
};

bool MCP9600::init(uint8_t bus, uint8_t address)
{
    _dev = std::move(hal.i2c_mgr->get_device(bus, address));
    if (!_dev) {
        printf("MCP9600 device is null!");
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    uint8_t buf[2];
    if (!_dev->read_registers(MCP9600_CMD_DEVICE_ID_REV, buf, 2)) {
        printf("MCP9600 failed to get WHOAMI");
        return false;
    }
    if (buf[0] != MCP9600_WHOAMI) {
        printf("MCP9600 Got wrong WHOAMI: detected 0x%2X but expected 0x%2X", (unsigned)buf[0], (unsigned)MCP9600_WHOAMI);
        return false;
    }
    if (!set_config(_thermoType_param, _filter_param)) {
        printf("MCP9600 unable to configure");
        return false;
    }

    _bus = bus;
    _address = address;
    _thermoType = _thermoType_param;
    _filter = _filter_param;

    printf("MCP9600 Detected! Rev %u.%u on bus: %u addr: 0x%2X", (unsigned)(buf[1] >> 4), (unsigned)(buf[1] & 0x0F), (unsigned)_bus, (unsigned)_address);

    // tick once to update temp and health
    _timer();

    // lower retries for run
    _dev->set_retries(3);

    /* Request 2Hz update */
    _dev->register_periodic_callback(2 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&MCP9600::_timer, void));
    return true;
}

void MCP9600::_timer(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    float temperature;
    if (read_temperature(temperature)) {
        WITH_SEMAPHORE(sem);
        _temperature = temperature;
        _healthy = true;
        _last_reading_ms = now_ms;

        // check for run-time changes
        if ((_filter != _filter_param) || (_thermoType != _thermoType_param)) {
            _thermoType = _thermoType_param;
            _filter = _filter_param;
            set_config(_thermoType, _filter);
        }

    } else if (now_ms - _last_reading_ms > 5000) {
        _healthy = false;
        _temperature = 0;
    }
}

bool MCP9600::set_config(const ThermocoupleType thermoType, const uint8_t filter)
{
    const uint8_t data = (uint8_t)thermoType << 4 | (filter & 0x7);
    if (!_dev->write_register(MCP9600_CMD_SENSOR_CFG, data)) {
        return false;
    }
    return true;
}

bool MCP9600::read_temperature(float &temperature)
{
    uint8_t data[2];
    if (!_dev->read_registers(MCP9600_CMD_HOT_JUNCT_TEMP, data, 2)) {
        return false;
    }

    const uint8_t data_swapped[2] = {data[1], data[0]};
    temperature = (float)*((int16_t*) data_swapped) * 0.0625f;
    return true;
}
#endif // HAL_TEMP_SENSOR_MCP9600_ENABLE

