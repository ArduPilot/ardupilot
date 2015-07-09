/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
/*
  BMP085 barometer driver. Based on original code by Jordi Munoz and
  Jose Julio

  Substantially modified by Andrew Tridgell
*/

#include <AP_HAL.h>
#include <AP_Common.h>

#include "AP_Baro.h"

extern const AP_HAL::HAL& hal;

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)

#define BMP085_CALIBRATION_DATA_START   0xAA
#define BMP085_CALIBRATION_DATA_LENGTH  22  /* 16 bit values */
#define BMP085_CTRL_REG                 0xF4
#define BMP085_TEMP_MEASUREMENT         0x2E
#define BMP085_PRESSURE_MEASUREMENT     0x34
#define BMP085_CONVERSION_REGISTER_MSB  0xF6
#define BMP085_CONVERSION_REGISTER_LSB  0xF7
#define BMP085_CONVERSION_REGISTER_XLSB 0xF8

// the apm2 hardware needs to check the state of the
// chip using a direct IO port
// On APM2 prerelease hw, the data ready port is hooked up to PE7, which
// is not available to the arduino digitalRead function.
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define BMP_DATA_READY() hal.gpio->read(BMP085_EOC)
// End of conversion pin PC7
#define BMP085_EOC 30
#else
// No EOC connection from Baro
// Use time instead:
//     Temperature conversion time is 4.5ms
//     Pressure conversion time is 25.5ms (for OVERSAMPLING=3)
#define BMP_DATA_READY() (_state == 0 ? hal.scheduler->millis() > (_last_temp_read_command_time + 5) : hal.scheduler->millis() > (_last_press_read_command_time + 26))
#define BMP085_EOC -1
#endif

// oversampling 3 gives 26ms conversion time. We then average
#define OVERSAMPLING 3

/*
  constructor
 */
AP_Baro_BMP085::AP_Baro_BMP085(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    uint8_t buff[BMP085_CALIBRATION_DATA_LENGTH];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("BMP085: unable to get semaphore"));
    }

    if (BMP085_EOC > 0) {
        // End Of Conversion input
        hal.gpio->pinMode(BMP085_EOC, HAL_GPIO_INPUT);
    }

    // We read the calibration data registers
    if (hal.i2c->readRegisters(BMP085_ADDRESS, BMP085_CALIBRATION_DATA_START,
                               BMP085_CALIBRATION_DATA_LENGTH, buff) != 0) {
        hal.scheduler->panic(PSTR("BMP085: bad calibration registers"));
    }

    _ac1 = ((int16_t)buff[0] << 8) | buff[1];
    _ac2 = ((int16_t)buff[2] << 8) | buff[3];
    _ac3 = ((int16_t)buff[4] << 8) | buff[5];
    _ac4 = ((int16_t)buff[6] << 8) | buff[7];
    _ac5 = ((int16_t)buff[8] << 8) | buff[9];
    _ac6 = ((int16_t)buff[10] << 8) | buff[11];
    _b1 = ((int16_t)buff[12] << 8) | buff[13];
    _b2 = ((int16_t)buff[14] << 8) | buff[15];
    _mb = ((int16_t)buff[16] << 8) | buff[17];
    _mc = ((int16_t)buff[18] << 8) | buff[19];
    _md = ((int16_t)buff[20] << 8) | buff[21];

    _last_press_read_command_time = 0;
    _last_temp_read_command_time = 0;

    _instance = _frontend.register_sensor();

    // Send a command to read temperature
    _command_read_temp();

    _state = 0;

    i2c_sem->give();
}

// Read the sensor. This is a state machine
// acumulate a new sensor reading
void AP_Baro_BMP085::accumulate(void)
{
    if (!BMP_DATA_READY()) {
        return;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(1))
        return;

    if (_state == 0) {
        _read_temp();
    } else {
        if (_read_press()) {
            _calculate();
        }
    }
    _state++;
    if (_state == 5) {
        _state = 0;
        _command_read_temp();
    } else {
        _command_read_press();
    }

    i2c_sem->give();
}


/*
  transfer data to the frontend
 */
void AP_Baro_BMP085::update(void)
{
    if (_count == 0) {
        accumulate();
    }
    if (_count == 0) {
        return;
    }

    float temperature = 0.1f * _temp_sum / _count;
    float pressure = _press_sum / _count;

    _count = 0;
    _temp_sum = 0;
    _press_sum = 0;

    _copy_to_frontend(_instance, pressure, temperature);
}

// Send command to Read Pressure
void AP_Baro_BMP085::_command_read_press()
{
    // Mode 0x34+(OVERSAMPLING << 6) is osrs=3 when OVERSAMPLING=3 => 25.5ms conversion time
    hal.i2c->writeRegister(BMP085_ADDRESS, BMP085_CTRL_REG,
                           BMP085_PRESSURE_MEASUREMENT + (OVERSAMPLING << 6));
    _last_press_read_command_time = hal.scheduler->millis();
}

// Read Raw Pressure values
bool AP_Baro_BMP085::_read_press()
{
    uint8_t buf[3];

    if (hal.i2c->readRegisters(BMP085_ADDRESS, BMP085_CONVERSION_REGISTER_MSB,
                               3, buf) != 0) {
        hal.i2c->setHighSpeed(false);
        return false;
    }

    _raw_press = ((uint32_t)buf[0] << 16)
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2]);
    _raw_press >>= (8 - OVERSAMPLING);

    return true;
}

// Send Command to Read Temperature
void AP_Baro_BMP085::_command_read_temp()
{
    hal.i2c->writeRegister(BMP085_ADDRESS, BMP085_CTRL_REG,
                           BMP085_TEMP_MEASUREMENT);
    _last_temp_read_command_time = hal.scheduler->millis();
}

// Read Raw Temperature values
void AP_Baro_BMP085::_read_temp()
{
    uint8_t buf[2];
    int32_t _temp_sensor;

    if (hal.i2c->readRegisters(BMP085_ADDRESS, BMP085_CONVERSION_REGISTER_MSB,
                               2, buf) != 0) {
        hal.i2c->setHighSpeed(false);
        return;
    }
    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    _raw_temp = _temp_sensor;
}


// Calculate Temperature and Pressure in real units.
void AP_Baro_BMP085::_calculate()
{
    int32_t x1, x2, x3, b3, b5, b6, p;
    uint32_t b4, b7;
    int32_t tmp;

    // See Datasheet page 13 for these formulas
    // Based also on Jee Labs BMP085 example code. Thanks for share.
    // Temperature calculations
    x1 = ((int32_t)_raw_temp - _ac6) * _ac5 >> 15;
    x2 = ((int32_t)_mc << 11) / (x1 + _md);
    b5 = x1 + x2;
    _temp_sum += (b5 + 8) >> 4;

    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (_b2 * (b6 * b6 >> 12)) >> 11;
    x2 = _ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = _ac1;
    tmp = (tmp*4 + x3)<<OVERSAMPLING;
    b3 = (tmp+2)/4;
    x1 = _ac3 * b6 >> 13;
    x2 = (_b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (_ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t)_raw_press - b3) * (50000 >> OVERSAMPLING);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    _press_sum += p + ((x1 + x2 + 3791) >> 4);

    _count++;
    if (_count == 254) {
        _temp_sum *= 0.5f;
        _press_sum *= 0.5f;
        _count /= 2;
    }
}
