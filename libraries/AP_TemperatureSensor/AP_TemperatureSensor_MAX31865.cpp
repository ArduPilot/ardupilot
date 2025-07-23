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


const AP_Param::GroupInfo AP_TemperatureSensor_MAX31865::var_info[] = {

    // @Param: RTD_NOM
    // @DisplayName: Nominal RTD resistance
    // @Description: Nominal RTD resistance used to calculate temperature, typically 100 or 1000 ohms.
    // @User: Standard
    AP_GROUPINFO("RTD_NOM", 8, AP_TemperatureSensor_MAX31865, nominal_resistance, 100),

    // @Param: RTD_REF
    // @DisplayName: RTD reference resistance
    // @Description: Reference resistance used to calculate temperature, in ohms
    // @User: Standard
    AP_GROUPINFO("RTD_REF", 9, AP_TemperatureSensor_MAX31865, reference_resistance, 400),

    // CHECK/UPDATE INDEX TABLE IN AP_TemperatureSensor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND

};

AP_TemperatureSensor_MAX31865::AP_TemperatureSensor_MAX31865(AP_TemperatureSensor &front,
                                                         AP_TemperatureSensor::TemperatureSensor_State &state,
                                                         AP_TemperatureSensor_Params &params) :
    AP_TemperatureSensor_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

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

    _dev = hal.spi->get_device_ptr(name);
    if (!_dev) {
        return;
    }

    // Set 3 wire config bit
    if (_params.type == AP_TemperatureSensor_Params::Type::MAX31865_3_wire) {
        config_register |= 0b00010000;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    _dev->write_register(MAX31865_REG_CONFIG_WRITE, config_register);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    /* Request 5Hz update */
    _dev->register_periodic_callback(200 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_MAX31865::thread_tick, void));
}

// Convert raw value in to temperature in deg celsius
// https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
float AP_TemperatureSensor_MAX31865::calculate_temperature(const uint16_t raw) const
{
    const float measured_resistance = (raw / 32768.0) * reference_resistance;

    // direct mathmatical method, valid for positive temperature
    const float A = 3.9083e-3;
    const float B = -5.775e-7;

    const float Z1 = -A;
    const float Z2 = A*A - (4.0 * B);
    const float Z3 = (4.0 * B) / nominal_resistance;
    const float Z4 = 2.0 * B;

    float temp = (Z1 + sqrtf(Z2 + (Z3 * measured_resistance))) / Z4;

    if (is_positive(temp)) {
        return temp;
    }

    // Polynomial for sub zero temperatures
    // normalize to 100 ohm
    const float norm_resistance = (measured_resistance / nominal_resistance) * 100.0;

    float rpoly = norm_resistance;
    temp = -242.02;
    temp += 2.2228 * norm_resistance;
    rpoly *= norm_resistance;
    temp += 2.5859e-3 * norm_resistance;
    rpoly *= norm_resistance;
    temp -= 4.8260e-6 * norm_resistance;
    rpoly *= norm_resistance;
    temp -= 2.8183e-8 * norm_resistance;
    rpoly *= norm_resistance;
    temp += 1.5243e-10 * norm_resistance;

    return temp;
}

void AP_TemperatureSensor_MAX31865::thread_tick()
{
    if (!is_positive(nominal_resistance) || !is_positive(reference_resistance)) {
        // calculate_temperature function will fall over with bad reference values
        return;
    }

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
        if (_dev->read_registers(MAX31865_REG_FAULT_STATUS_READ, (uint8_t *)&data_fault, 1)) {
            Debug("%d MAX31865 fault: %u -> 0x%x", _state.instance, data, data_fault);
        } else {
            Debug("%d MAX31865 unkown fault: %u", data, _state.instance);
        }
#endif
        // clear the fault
        _dev->write_register(MAX31865_REG_CONFIG_WRITE, config_register);
        return;
    }

    const float temperature = calculate_temperature(data_temperature);

#if MAX31865_DEBUGGING
    Debug("%d MAX31865 %u -> %.2f C", _state.instance, data_temperature, temperature);
#endif

    set_temperature(temperature);
}

#endif // AP_TEMPERATURE_SENSOR_MAX31865_ENABLED

