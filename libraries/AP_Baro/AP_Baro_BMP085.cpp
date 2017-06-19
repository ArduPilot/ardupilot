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
#include "AP_Baro_BMP085.h"

#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

#define BMP085_OVERSAMPLING_ULTRALOWPOWER 0
#define BMP085_OVERSAMPLING_STANDARD      1
#define BMP085_OVERSAMPLING_HIGHRES       2
#define BMP085_OVERSAMPLING_ULTRAHIGHRES  3

#ifndef BMP085_EOC
#define BMP085_EOC -1
#define OVERSAMPLING BMP085_OVERSAMPLING_ULTRAHIGHRES
#else
#define OVERSAMPLING BMP085_OVERSAMPLING_HIGHRES
#endif

AP_Baro_BMP085::AP_Baro_BMP085(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
    uint8_t buff[22];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *sem = _dev->get_semaphore();

    // take i2c bus sempahore
    if (!sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("BMP085: unable to get semaphore");
    }

    if (BMP085_EOC >= 0) {
        _eoc = hal.gpio->channel(BMP085_EOC);
        _eoc->mode(HAL_GPIO_INPUT);
    }

    // We read the calibration data registers
    if (!_dev->read_registers(0xAA, buff, sizeof(buff))) {
        AP_HAL::panic("BMP085: bad calibration registers");
    }

    ac1 = ((int16_t)buff[0] << 8) | buff[1];
    ac2 = ((int16_t)buff[2] << 8) | buff[3];
    ac3 = ((int16_t)buff[4] << 8) | buff[5];
    ac4 = ((int16_t)buff[6] << 8) | buff[7];
    ac5 = ((int16_t)buff[8] << 8) | buff[9];
    ac6 = ((int16_t)buff[10] << 8) | buff[11];
    b1 = ((int16_t)buff[12] << 8) | buff[13];
    b2 = ((int16_t)buff[14] << 8) | buff[15];
    mb = ((int16_t)buff[16] << 8) | buff[17];
    mc = ((int16_t)buff[18] << 8) | buff[19];
    md = ((int16_t)buff[20] << 8) | buff[21];

    _last_press_read_command_time = 0;
    _last_temp_read_command_time = 0;

    _instance = _frontend.register_sensor();

    // Send a command to read temperature
    _cmd_read_temp();

    _state = 0;

    sem->give();

    _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_Baro_BMP085::_timer, void));
}

/*
  This is a state machine. Acumulate a new sensor reading.
 */
void AP_Baro_BMP085::_timer(void)
{
    if (!_data_ready()) {
        return;
    }

    if (_state == 0) {
        _read_temp();
    } else if (_read_pressure()) {
        _calculate();
    }

    _state++;
    if (_state == 25) {
        _state = 0;
        _cmd_read_temp();
    } else {
        _cmd_read_pressure();
    }
}

/*
  transfer data to the frontend
 */
void AP_Baro_BMP085::update(void)
{
    if (_sem->take_nonblocking()) {
        if (!_has_sample) {
            _sem->give();
            return;
        }

        float temperature = 0.1f * _temp;
        float pressure = _pressure_filter.getf();

        _copy_to_frontend(_instance, pressure, temperature);
        _sem->give();
    }
}

// Send command to Read Pressure
void AP_Baro_BMP085::_cmd_read_pressure()
{
    _dev->write_register(0xF4, 0x34 + (OVERSAMPLING << 6));
    _last_press_read_command_time = AP_HAL::millis();
}

// Read raw pressure values
bool AP_Baro_BMP085::_read_pressure()
{
    uint8_t buf[3];

    if (!_dev->read_registers(0xF6, buf, sizeof(buf))) {
        _retry_time = AP_HAL::millis() + 1000;
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        return false;
    }

    _raw_pressure = (((uint32_t)buf[0] << 16)
                     | ((uint32_t)buf[1] << 8)
                     | ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
    return true;
}

// Send Command to Read Temperature
void AP_Baro_BMP085::_cmd_read_temp()
{
    _dev->write_register(0xF4, 0x2E);
    _last_temp_read_command_time = AP_HAL::millis();
}

// Read raw temperature values
void AP_Baro_BMP085::_read_temp()
{
    uint8_t buf[2];
    int32_t _temp_sensor;

    if (!_dev->read_registers(0xF6, buf, sizeof(buf))) {
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        return;
    }
    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    _raw_temp = _temp_sensor;
}

// _calculate Temperature and Pressure in real units.
void AP_Baro_BMP085::_calculate()
{
    int32_t x1, x2, x3, b3, b5, b6, p;
    uint32_t b4, b7;
    int32_t tmp;

    // See Datasheet page 13 for this formulas
    // Based also on Jee Labs BMP085 example code. Thanks for share.
    // Temperature calculations
    x1 = ((int32_t)_raw_temp - ac6) * ac5 >> 15;
    x2 = ((int32_t) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    _temp = (b5 + 8) >> 4;

    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    //b3 = (((int32_t) ac1 * 4 + x3)<<OVERSAMPLING + 2) >> 2; // BAD
    //b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for OVERSAMPLING=0
    tmp = ac1;
    tmp = (tmp*4 + x3)<<OVERSAMPLING;
    b3 = (tmp+2)/4;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) _raw_pressure - b3) * (50000 >> OVERSAMPLING);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += ((x1 + x2 + 3791) >> 4);

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _pressure_filter.apply(p);
        _has_sample = true;
        _sem->give();
    }
}

bool AP_Baro_BMP085::_data_ready()
{
    if (BMP085_EOC >= 0) {
        return _eoc->read();
    }

    // No EOC pin: use time from last read instead.
    if (_state == 0) {
        return AP_HAL::millis() > _last_temp_read_command_time + 5;
    }

    uint32_t conversion_time_msec;

    switch (OVERSAMPLING) {
    case BMP085_OVERSAMPLING_ULTRALOWPOWER:
        conversion_time_msec = 5;
        break;
    case BMP085_OVERSAMPLING_STANDARD:
        conversion_time_msec = 8;
        break;
    case BMP085_OVERSAMPLING_HIGHRES:
        conversion_time_msec = 14;
        break;
    case BMP085_OVERSAMPLING_ULTRAHIGHRES:
        conversion_time_msec = 26;
    default:
        break;
    }

    return AP_HAL::millis() > _last_press_read_command_time + conversion_time_msec;
}
