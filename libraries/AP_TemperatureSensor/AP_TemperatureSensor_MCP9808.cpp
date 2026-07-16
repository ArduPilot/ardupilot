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

#include "AP_TemperatureSensor_MCP9808.h"

#if AP_TEMPERATURE_SENSOR_MCP9808_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;


// MCP9808 register addresses

#define MCP9808_REG_CONFIG          0x01    // configuration register
#define MCP9808_REG_AMBIENT_TEMP    0x05    // ambient temperature register
#define MCP9808_REG_DEVICE_ID       0x07    // device ID register


// MCP9808 device identification

#define MCP9808_DEVICE_ID           0x0400


// Configuration register
//
// Continuous conversion mode
// No shutdown
// Default resolution (0.0625 C)

#define MCP9808_CONFIG_VALUE        0x0000


void AP_TemperatureSensor_MCP9808::init()
{
    _params.bus_address.set_default(
        AP_TEMPERATURE_SENSOR_MCP9808_DEFAULT_I2C_ADDR);


    _dev = hal.i2c_mgr->get_device_ptr(
        _params.bus,
        _params.bus_address);


    if (!_dev) {
        return;
    }


    WITH_SEMAPHORE(_dev->get_semaphore());


    // Increase retries during startup
    _dev->set_retries(10);


    // Confirm we are talking to a MCP9808
    uint16_t device_id;

    if (!read_registers(MCP9808_REG_DEVICE_ID,device_id) || device_id != MCP9808_DEVICE_ID) {
        return;
    }


    // Configure sensor for continuous conversion
    if (!write_register(MCP9808_REG_CONFIG,MCP9808_CONFIG_VALUE)) {
        return;
    }

    // Reduce retries during normal operation
    _dev->set_retries(3);

    // Poll temperature at 20Hz
    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_MCP9808::_timer,void));
}



void AP_TemperatureSensor_MCP9808::_timer(void)
{
    uint16_t raw;


    if (!read_registers(
            MCP9808_REG_AMBIENT_TEMP,
            raw)) {
        return;
    }


    /*
        MCP9808 temperature register format:

        bit15     : Alert flag
        bit14-13  : Temperature alert status
        bit12     : Sign bit
        bit11-0   : Temperature data

        Resolution:
        1 LSB = 0.0625 C

    */


    float temp = raw & 0x0FFF;


    temp *= 0.0625f;


    // Negative temperature conversion

    if (raw & 0x1000) {
        temp -= 256.0f;
    }

    set_temperature(temp);
}


bool AP_TemperatureSensor_MCP9808::read_registers(
    uint8_t reg,
    uint16_t &value) const
{
    uint8_t val[2];


    if (!_dev->transfer(&reg,1,val,sizeof(val))) {
        return false;
    }

    // MCP9808 registers are 16-bit big endian

    value = UINT16_VALUE(val[0],val[1]);
    return true;
}


bool AP_TemperatureSensor_MCP9808::write_register(uint8_t reg,uint16_t value) const
{
    // Registers are 16-bit big endian

    uint8_t buf[3] {reg,uint8_t(value >> 8),uint8_t(value & 0xFF)};
    return _dev->transfer(buf,sizeof(buf),nullptr,0);
}


#endif // AP_TEMPERATURE_SENSOR_MCP9808_ENABLED