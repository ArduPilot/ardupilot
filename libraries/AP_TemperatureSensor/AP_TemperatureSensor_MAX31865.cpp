/*
 * Copyright (C) 2023 Kraus Hamdani Aerospace Inc. All rights reserved.
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
    Implements SPI driver for MAX31865 digital RTD Temperature converter
*/

#include "AP_TemperatureSensor_MAX31865.h"

#if AP_TEMPERATURE_SENSOR_MAX31865_ENABLED
#include <stdio.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

#define MAX31865_REG_WRITE_ADDR_OFFSET      0x80


#define MAX31865_REG_CONFIG_READ            0x00
#define MAX31865_REG_CONFIG_WRITE           (MAX31865_REG_CONFIG_READ | MAX31865_REG_WRITE_ADDR_OFFSET)
#define MAX31865_REG_DATA_MSB               0x01
#define MAX31865_REG_DATA_LSB               0x02
#define MAX31865_REG_HIGH_FAULT_THRESH_MSB  0x03
#define MAX31865_REG_HIGH_FAULT_THRESH_LSB  0x04
#define MAX31865_REG_LOW_FAULT_THRESH_MSB   0x05
#define MAX31865_REG_LOW_FAULT_THRESH_LSB   0x06
#define MAX31865_REG_FAULT_STATUS_READ      0x07
#define MAX31865_REG_FAULT_STATUS_WRITE     (MAX31865_REG_FAULT_STATUS_READ | MAX31865_REG_WRITE_ADDR_OFFSET)


// Vbias = ON (must be on for automatic mode)
// Conversion mode: Auto
// Fault Status: 1set 1 to Clear all latched faults
#define MAX31865_CONFIG_VALUE               (0b11000010)

#define MAX31865_DEBUGGING 0

#if MAX31865_DEBUGGING
#ifdef HAL_BUILD_AP_PERIPH
    extern "C" {
    void can_printf(const char *fmt, ...);
    }
    //# define Debug(fmt, args ...)  do { can_printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
      #  define Debug(fmt, args ...)  do { can_printf(fmt, ## args); } while(0)
#else
 # define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
 #endif // HAL_BUILD_AP_PERIPH
#else
 # define Debug(fmt, args ...)
#endif // MAX31865_DEBUGGING


void AP_TemperatureSensor_MAX31865::init()
{
    if (_dev) {
        return;
    }

    // also look for "max31865_n" where n is the addr param.
    char name[12] = "max31865";
    if (_params.bus_address > 0 && _params.bus_address <= 9) {
        name[8] = '_';
        name[9] = '0' + _params.bus_address; // convert bus_address to ascii
        name[10] = 0;
    }

    _dev = std::move(hal.spi->get_device(name));
    if (!_dev) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    _dev->write_register(MAX31865_REG_CONFIG_WRITE, MAX31865_CONFIG_VALUE);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    /* Request 5Hz update */
    _dev->register_periodic_callback(200 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_MAX31865::thread_tick, void));
}

void AP_TemperatureSensor_MAX31865::thread_tick()
{
    uint16_t raw_data;
    if (!_dev->read_registers(MAX31865_REG_DATA_MSB, (uint8_t *)&raw_data, sizeof(raw_data))) {
        return;
    }

    // 16bit byte swap
     const uint16_t data = htobe16(raw_data);

    // fault is LSB bit, temperature data is upper 15 bits
    const bool is_fault = (data & 0x0001) != 0;
    const uint16_t data_temperature = data >> 1;

    if (is_fault) {
#if MAX31865_DEBUGGING
        uint8_t data_fault = 0;
        _dev->read_registers(MAX31865_REG_FAULT_STATUS_READ, (uint8_t *)&data_fault, 1);
        Debug("%d MAX31865 fault: 0x02X", _state.instance, data_fault);
#endif
        // clear the fault
        _dev->write_register(MAX31865_REG_CONFIG_WRITE, MAX31865_CONFIG_VALUE);
        return;
    }

    // For a temperature range of -100 C to +100 C, a good approximation of
    // temperature can be made with simple linearization. This equation gives
    // 0 C error at 0C, -1.75 C error at -100 C, and -1.4 C error at +100 C
    const float temperature = ((float)data_temperature/32.0f) - 256.0f;

#if MAX31865_DEBUGGING
    Debug("%d MAX31865 %.2f C", _state.instance, temperature);
#endif

    set_temperature(temperature);
}

#endif // AP_TEMPERATURE_SENSOR_MAX31865_ENABLED

