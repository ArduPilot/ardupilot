/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       APM_BMP085.cpp - Arduino Library for BMP085 absolute pressure sensor
 *       Code by Jordi Mu�oz and Jose Julio. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Sensor is conected to I2C port
 *       Sensor End of Conversion (EOC) pin is PC7 (30)
 *
 *       Variables:
 *               RawTemp : Raw temperature data
 *               RawPress : Raw pressure data
 *
 *               Temp : Calculated temperature (in 0.1�C units)
 *               Press : Calculated pressure   (in Pa units)
 *
 *       Methods:
 *               Init() : Initialization of I2C and read sensor calibration data
 *               Read() : Read sensor data and calculate Temperature and Pressure
 *                        This function is optimized so the main host don�t need to wait
 *                                You can call this function in your main loop
 *                                It returns a 1 if there are new data.
 *
 *       Internal functions:
 *               Command_ReadTemp(): Send commando to read temperature
 *               Command_ReadPress(): Send commando to read Pressure
 *               ReadTemp() : Read temp register
 *               ReadPress() : Read press register
 *               Calculate() : Calculate Temperature and Pressure in real units
 *
 *
 */

// AVR LibC Includes
#include <inttypes.h>

#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library

#include <AP_HAL.h>

#include "AP_Baro_BMP085.h"

extern const AP_HAL::HAL& hal;

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)
#define BMP085_EOC 30        // End of conversion pin PC7

// the apm2 hardware needs to check the state of the
// chip using a direct IO port
// On APM2 prerelease hw, the data ready port is hooked up to PE7, which
// is not available to the arduino digitalRead function.
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define BMP_DATA_READY() (PINE&0x80)
#else
#define BMP_DATA_READY() hal.gpio->read(BMP085_EOC)
#endif

// oversampling 3 gives highest resolution
#define OVERSAMPLING 3

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_BMP085::init()
{
    uint8_t buff[22];

    hal.gpio->pinMode(BMP085_EOC, GPIO_INPUT);// End Of Conversion (PC7) input

    BMP085_State = 0;                    // Initial state

    // We read the calibration data registers
    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xAA, 22, buff) != 0) {
        healthy = false;
        return false;
    }

    hal.console->println_P(PSTR("read bmp085 calibration regs"));

    ac1 = ((int)buff[0] << 8) | buff[1];
    ac2 = ((int)buff[2] << 8) | buff[3];
    ac3 = ((int)buff[4] << 8) | buff[5];
    ac4 = ((int)buff[6] << 8) | buff[7];
    ac5 = ((int)buff[8] << 8) | buff[9];
    ac6 = ((int)buff[10] << 8) | buff[11];
    b1 = ((int)buff[12] << 8) | buff[13];
    b2 = ((int)buff[14] << 8) | buff[15];
    mb = ((int)buff[16] << 8) | buff[17];
    mc = ((int)buff[18] << 8) | buff[19];
    md = ((int)buff[20] << 8) | buff[21];

    //Send a command to read Temp
    Command_ReadTemp();
    
    hal.console->println_P(PSTR("read bmp085 temp"));

    BMP085_State = 1;

    // init raw temo
    RawTemp = 0;

    healthy = true;
    return true;
}

// Read the sensor. This is a state machine
// We read Temperature (state=1) and then Pressure (state!=1) on alternate calls
uint8_t AP_Baro_BMP085::read()
{
    uint8_t result = 0;

    if (BMP085_State == 1) {
        if (BMP_DATA_READY()) {
            BMP085_State = 2;
            ReadTemp();                                                          // On state 1 we read temp
            Command_ReadPress();
        }
    }else{
        if (BMP_DATA_READY()) {
            BMP085_State = 1;                                   // Start again from state = 1
            ReadPress();
            Calculate();
            Command_ReadTemp();                                 // Read Temp
            result = 1;                                                 // New pressure reading
        }
    }
    if (result) {
        _last_update = hal.scheduler->millis();
    }
    return(result);
}

float AP_Baro_BMP085::get_pressure() {
    return Press;
}

float AP_Baro_BMP085::get_temperature() {
    return Temp;
}

int32_t AP_Baro_BMP085::get_raw_pressure() {
    return RawPress;
}

int32_t AP_Baro_BMP085::get_raw_temp() {
    return RawTemp;
}

// Private functions: /////////////////////////////////////////////////////////

// Send command to Read Pressure
void AP_Baro_BMP085::Command_ReadPress()
{
    uint8_t res = hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4,
            0x34+(OVERSAMPLING << 6));
    if (res != 0) {
        healthy = false;
    }
}

// Read Raw Pressure values
void AP_Baro_BMP085::ReadPress()
{
    uint8_t buf[3];

    if (!healthy && hal.scheduler->millis() < _retry_time) {
        return;
    }

    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 3, buf) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        hal.i2c->setHighSpeed(false);
        healthy = false;
        return;
    }

    RawPress = (((uint32_t)buf[0] << 16) 
             | ((uint32_t)buf[1] << 8)
             | ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
}

// Send Command to Read Temperature
void AP_Baro_BMP085::Command_ReadTemp()
{
    if (hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x2E) != 0) {
        healthy = false;
    }
}

// Read Raw Temperature values
void AP_Baro_BMP085::ReadTemp()
{
    uint8_t buf[2];
    int32_t _temp_sensor;

    if (!healthy && hal.scheduler->millis() < _retry_time) {
        return;
    }

    if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 2, buf) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        hal.i2c->setHighSpeed(false);
        healthy = false;
        return;
    }
    _temp_sensor = buf[0];
    _temp_sensor = (_temp_sensor << 8) | buf[1];

    RawTemp = _temp_filter.apply(_temp_sensor);
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
    Temp = (b5 + 8) >> 4;

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
    Press = p + ((x1 + x2 + 3791) >> 4);
}
