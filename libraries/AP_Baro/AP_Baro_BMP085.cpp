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
#define BMP085_EOC 30        // End of conversion pin PC7 on APM1

// the apm2 hardware needs to check the state of the
// chip using a direct IO port
// On APM2 prerelease hw, the data ready port is hooked up to PE7, which
// is not available to the arduino digitalRead function.
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define BMP_DATA_READY() hal.gpio->read(BMP085_EOC)
#else
// No EOC connection from Baro
// Use times instead.
// Temp conversion time is 4.5ms
// Pressure conversion time is 25.5ms (for OVERSAMPLING=3)
#define BMP_DATA_READY() (BMP085_State == 0 ? hal.scheduler->millis() > (_last_temp_read_command_time + 5) : hal.scheduler->millis() > (_last_press_read_command_time + 26))
#endif

// oversampling 3 gives 26ms conversion time. We then average
#define OVERSAMPLING 3

/*
  constructor
 */
AP_Baro_BMP085::AP_Baro_BMP085(AP_Baro &baro) :
    AP_Baro_Backend(baro),
    _instance(0),
    _temp_sum(0),
    _press_sum(0),
    _count(0),
    BMP085_State(0),
    ac1(0), ac2(0), ac3(0), b1(0), b2(0), mb(0), mc(0), md(0),
    ac4(0), ac5(0), ac6(0),
    _retry_time(0)
{
    uint8_t buff[22];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("BMP085: unable to get semaphore"));
    }

    // End Of Conversion (PC7) input
    hal.gpio->pinMode(BMP085_EOC, HAL_GPIO_INPUT);

    // We read the calibration data registers
    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xAA, 22, buff) != 0) {
        hal.scheduler->panic(PSTR("BMP085: bad calibration registers"));
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

    //Send a command to read Temp
    Command_ReadTemp();
    
    BMP085_State = 0;

    i2c_sem->give();
}

// Read the sensor. This is a state machine
// acumulate a new sensor reading
void AP_Baro_BMP085::accumulate(void)
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    if (!BMP_DATA_READY()) {
        return;
    }

    // take i2c bus sempahore
    if (!i2c_sem->take(1))
        return;

    if (BMP085_State == 0) {
        ReadTemp();
    } else {
        if (ReadPress()) {
            Calculate();
        }
    }
    BMP085_State++;
    if (BMP085_State == 5) {
        BMP085_State = 0;
        Command_ReadTemp();
    } else {
        Command_ReadPress();
    }

    i2c_sem->give();
}


/*
  transfer data to the frontend
 */
void AP_Baro_BMP085::update(void)
{
    if (_count == 0 && BMP_DATA_READY()) {
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
void AP_Baro_BMP085::Command_ReadPress()
{
    // Mode 0x34+(OVERSAMPLING << 6) is osrs=3 when OVERSAMPLING=3 => 25.5ms conversion time
    hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4,
                           0x34+(OVERSAMPLING << 6));
    _last_press_read_command_time = hal.scheduler->millis();
}

// Read Raw Pressure values
bool AP_Baro_BMP085::ReadPress()
{
    uint8_t buf[3];

    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 3, buf) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        hal.i2c->setHighSpeed(false);
        return false;
    }

    RawPress = (((uint32_t)buf[0] << 16) 
             | ((uint32_t)buf[1] << 8)
             | ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
    return true;
}

// Send Command to Read Temperature
void AP_Baro_BMP085::Command_ReadTemp()
{
    hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x2E);
    _last_temp_read_command_time = hal.scheduler->millis();
}

// Read Raw Temperature values
void AP_Baro_BMP085::ReadTemp()
{
    uint8_t buf[2];
    int32_t _temp_sensor;

    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 2, buf) != 0) {
        hal.i2c->setHighSpeed(false);
        return;
    }
    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    RawTemp = _temp_sensor;
}


// Calculate Temperature and Pressure in real units.
void AP_Baro_BMP085::Calculate()
{
    int32_t x1, x2, x3, b3, b5, b6, p;
    uint32_t b4, b7;
    int32_t tmp;

    // See Datasheet page 13 for this formulas
    // Based also on Jee Labs BMP085 example code. Thanks for share.
    // Temperature calculations
    x1 = ((int32_t)RawTemp - ac6) * ac5 >> 15;
    x2 = ((int32_t) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    _temp_sum += (b5 + 8) >> 4;

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
    b7 = ((uint32_t) RawPress - b3) * (50000 >> OVERSAMPLING);
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
