/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       APM_MS5611.cpp - Arduino Library for MS5611-01BA01 absolute pressure sensor
 *       Code by Jose Julio, Pat Hickey and Jordi Muñoz. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       Sensor is conected to standard SPI port
 *       Chip Select pin: Analog2 (provisional until Jordi defines the pin)!!
 *
 *       Variables:
 *               Temp : Calculated temperature (in Celsius degrees * 100)
 *               Press : Calculated pressure   (in mbar units * 100)
 *
 *
 *       Methods:
 *               init() : Initialization and sensor reset
 *               read() : Read sensor data and _calculate Temperature, Pressure
 *                        This function is optimized so the main host don´t need to wait
 *                                You can call this function in your main loop
 *                                Maximum data output frequency 100Hz - this allows maximum oversampling in the chip ADC
 *                                It returns a 1 if there are new data.
 *               get_pressure() : return pressure in mbar*100 units
 *               get_temperature() : return temperature in celsius degrees*100 units
 *
 *       Internal functions:
 *               _calculate() : Calculate Temperature and Pressure (temperature compensated) in real units
 *
 *
 */

#include <FastSerial.h>
#include <SPI.h>
#include "AP_Baro_MS5611.h"


/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40) */
#define MS5611_CS 40

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

uint32_t volatile AP_Baro_MS5611::_s_D1;
uint32_t volatile AP_Baro_MS5611::_s_D2;
uint8_t volatile AP_Baro_MS5611::_d1_count;
uint8_t volatile AP_Baro_MS5611::_d2_count;
uint8_t AP_Baro_MS5611::_state;
uint32_t AP_Baro_MS5611::_timer;
bool AP_Baro_MS5611::_sync_access;
bool volatile AP_Baro_MS5611::_updated;

uint8_t AP_Baro_MS5611::_spi_read(uint8_t reg)
{
    uint8_t return_value;
    uint8_t addr = reg; // | 0x80; // Set most significant bit
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    return_value = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return return_value;
}

uint16_t AP_Baro_MS5611::_spi_read_16bits(uint8_t reg)
{
    uint8_t byteH, byteL;
    uint16_t return_value;
    uint8_t addr = reg; // | 0x80; // Set most significant bit
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    byteH = SPI.transfer(0);
    byteL = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return_value = ((uint16_t)byteH<<8) | (byteL);
    return return_value;
}

uint32_t AP_Baro_MS5611::_spi_read_adc()
{
    uint8_t byteH,byteM,byteL;
    uint32_t return_value;
    uint8_t addr = 0x00;
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    byteH = SPI.transfer(0);
    byteM = SPI.transfer(0);
    byteL = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
    return return_value;
}


void AP_Baro_MS5611::_spi_write(uint8_t reg)
{
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(reg); // discarded
    digitalWrite(MS5611_CS, HIGH);
}

// Public Methods //////////////////////////////////////////////////////////////
// SPI should be initialized externally
bool AP_Baro_MS5611::init( AP_PeriodicProcess *scheduler )
{
    scheduler->suspend_timer();

    pinMode(MS5611_CS, OUTPUT);          // Chip select Pin
    digitalWrite(MS5611_CS, HIGH);
    delay(1);

    _spi_write(CMD_MS5611_RESET);
    delay(4);

    // We read the factory calibration
    // The on-chip CRC is not used
    C1 = _spi_read_16bits(CMD_MS5611_PROM_C1);
    C2 = _spi_read_16bits(CMD_MS5611_PROM_C2);
    C3 = _spi_read_16bits(CMD_MS5611_PROM_C3);
    C4 = _spi_read_16bits(CMD_MS5611_PROM_C4);
    C5 = _spi_read_16bits(CMD_MS5611_PROM_C5);
    C6 = _spi_read_16bits(CMD_MS5611_PROM_C6);


    //Send a command to read Temp first
    _spi_write(CMD_CONVERT_D2_OSR4096);
    _timer = micros();
    _state = 0;
    Temp=0;
    Press=0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

    scheduler->resume_timer();
    scheduler->register_process( AP_Baro_MS5611::_update );

    // wait for at least one value to be read
    while (!_updated) ;

    healthy = true;
    return true;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void AP_Baro_MS5611::_update(uint32_t tnow)
{
    if (_sync_access) return;

    // Throttle read rate to 100hz maximum.
    // note we use 9500us here not 10000us
    // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
    if (tnow - _timer < 9500) {
        return;
    }

    _timer = tnow;

    if (_state == 0) {
        _s_D2 += _spi_read_adc();                                // On state 0 we read temp
        _d2_count++;
        if (_d2_count == 32) {
            // we have summed 32 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D2 >>= 1;
            _d2_count = 16;
        }
        _state++;
        _spi_write(CMD_CONVERT_D1_OSR4096);      // Command to read pressure
    } else {
        _s_D1 += _spi_read_adc();
        _d1_count++;
        if (_d1_count == 128) {
            // we have summed 128 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D1 >>= 1;
            _d1_count = 64;
        }
        _state++;
        _updated = true;                                                        // New pressure reading
        if (_state == 5) {
            _spi_write(CMD_CONVERT_D2_OSR4096); // Command to read temperature
            _state = 0;
        } else {
            _spi_write(CMD_CONVERT_D1_OSR4096); // Command to read pressure
        }
    }
}

uint8_t AP_Baro_MS5611::read()
{
    _sync_access = true;
    bool updated = _updated;
    if (updated) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;
        // we need to disable interrupts to access
        // _s_D1 and _s_D2 as they are not atomic
        cli();
        sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;
        _updated = false;
        sei();
        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = ((float)sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }
    _sync_access = false;
    _calculate();
    if (updated) {
        _last_update = millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5611::_calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;
    float P;

    // Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of D1 and D1 over
    // multiple samples, giving us more precision
    dT = D2-(((uint32_t)C5)<<8);
    TEMP = (dT * C6)/8388608;
    OFF = C2 * 65536.0 + (C4 * dT) / 128;
    SENS = C1 * 32768.0 + (C3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5*Aux;
        float SENS2 = 1.25*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P = (D1*SENS/2097152 - OFF)/32768;
    Temp = TEMP + 2000;
    Press = P;
}

float AP_Baro_MS5611::get_pressure()
{
    return Press;
}

float AP_Baro_MS5611::get_temperature()
{
    // callers want the temperature in 0.1C units
    return Temp/10;
}

int32_t AP_Baro_MS5611::get_raw_pressure() {
    return _raw_press;
}

int32_t AP_Baro_MS5611::get_raw_temp() {
    return _raw_temp;
}


