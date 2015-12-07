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
  originally written by Jose Julio, Pat Hickey and Jordi Muñoz

  Heavily modified by Andrew Tridgell
*/
#include "AP_Baro_MS5611.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define CMD_MS5611_RESET 0x1E
#define CMD_MS56XX_PROM 0xA0

#define ADDR_CMD_CONVERT_D1_OSR256  0x40 /* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512  0x42 /* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44 /* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46 /* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48 /* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50 /* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512  0x52 /* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54 /* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56 /* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58 /* write to this address to start temperature conversion */

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

// SPI Device //////////////////////////////////////////////////////////////////

AP_SerialBus_SPI::AP_SerialBus_SPI(enum AP_HAL::SPIDevice device, enum AP_HAL::SPIDeviceDriver::bus_speed speed) :
    _device(device),
    _speed(speed),
    _spi(NULL),
    _spi_sem(NULL)
{
}

void AP_SerialBus_SPI::init()
{
    _spi = hal.spi->device(_device);
    if (_spi == NULL) {
        AP_HAL::panic("did not get valid SPI device driver!");
    }
    _spi_sem = _spi->get_semaphore();
    if (_spi_sem == NULL) {
        AP_HAL::panic("AP_SerialBus_SPI did not get valid SPI semaphroe!");
    }
    _spi->set_bus_speed(_speed);
}

uint16_t AP_SerialBus_SPI::read_16bits(uint8_t reg)
{
    uint8_t tx[3] = { reg, 0, 0 };
    uint8_t rx[3];
    _spi->transaction(tx, rx, 3);
    return ((uint16_t) rx[1] << 8 ) | ( rx[2] );
}

uint32_t AP_SerialBus_SPI::read_24bits(uint8_t reg)
{
    uint8_t tx[4] = { reg, 0, 0, 0 };
    uint8_t rx[4];
    _spi->transaction(tx, rx, 4);
    return (((uint32_t)rx[1])<<16) | (((uint32_t)rx[2])<<8) | ((uint32_t)rx[3]);
}

bool AP_SerialBus_SPI::write(uint8_t reg)
{
    uint8_t tx[1] = { reg };
    _spi->transaction(tx, NULL, 1);
    return true;
}

bool AP_SerialBus_SPI::sem_take_blocking() 
{
    return _spi_sem->take(10);
}

bool AP_SerialBus_SPI::sem_take_nonblocking()
{
    return _spi_sem->take_nonblocking();
}

void AP_SerialBus_SPI::sem_give()
{
    _spi_sem->give();
}


/// I2C SerialBus
AP_SerialBus_I2C::AP_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr) :
    _i2c(i2c),
    _addr(addr),
    _i2c_sem(NULL) 
{
}

void AP_SerialBus_I2C::init()
{
    _i2c_sem = _i2c->get_semaphore();
    if (_i2c_sem == NULL) {
        AP_HAL::panic("AP_SerialBus_I2C did not get valid I2C semaphore!");
    }
}

uint16_t AP_SerialBus_I2C::read_16bits(uint8_t reg)
{
    uint8_t buf[2];
    if (_i2c->readRegisters(_addr, reg, sizeof(buf), buf) == 0) {
        return (((uint16_t)(buf[0]) << 8) | buf[1]);
    }
    return 0;
}

uint32_t AP_SerialBus_I2C::read_24bits(uint8_t reg)
{
    uint8_t buf[3];
    if (_i2c->readRegisters(_addr, reg, sizeof(buf), buf) == 0) {
        return (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | buf[2];
    }
    return 0;
}

bool AP_SerialBus_I2C::write(uint8_t reg)
{
    return _i2c->write(_addr, 1, &reg) == 0;
}

bool AP_SerialBus_I2C::sem_take_blocking() 
{
    return _i2c_sem->take(10);
}

bool AP_SerialBus_I2C::sem_take_nonblocking()
{
    return _i2c_sem->take_nonblocking();
}

void AP_SerialBus_I2C::sem_give()
{
    _i2c_sem->give();
}

/*
  constructor
 */
AP_Baro_MS56XX::AP_Baro_MS56XX(AP_Baro &baro, AP_SerialBus *serial, bool use_timer) :
    AP_Baro_Backend(baro),
    _serial(serial),
    _updated(false),
    _state(0),
    _last_timer(0),
    _use_timer(use_timer),
    _D1(0.0f),
    _D2(0.0f)
{
}

void AP_Baro_MS56XX::_init()
{
    _instance = _frontend.register_sensor();
    _serial->init();

    // we need to suspend timers to prevent other SPI drivers grabbing
    // the bus while we do the long initialisation
    hal.scheduler->suspend_timer_procs();

    if (!_serial->sem_take_blocking()){
        AP_HAL::panic("PANIC: AP_Baro_MS56XX: failed to take serial semaphore for init");
    }

    _serial->write(CMD_MS5611_RESET);
    hal.scheduler->delay(4);

    uint16_t prom[8];
    if (!_read_prom(prom)) {
        AP_HAL::panic("Can't read PROM");
    }

    // Save factory calibration coefficients
    _c1 = prom[1];
    _c2 = prom[2];
    _c3 = prom[3];
    _c4 = prom[4];
    _c5 = prom[5];
    _c6 = prom[6];

    // Send a command to read Temp first
    _serial->write(ADDR_CMD_CONVERT_D2);
    _last_timer = AP_HAL::micros();
    _state = 0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

    _serial->sem_give();

    hal.scheduler->resume_timer_procs();

    if (_use_timer) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_MS56XX::_timer, void));
    }
}

/**
 * MS56XX crc4 method from datasheet for 16 bytes (8 short values)
 */
static uint16_t crc4(uint16_t *data)
{
    uint16_t n_rem = 0;
    uint8_t n_bit;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((data[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(data[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    return (n_rem >> 12) & 0xF;
}

bool AP_Baro_MS56XX::_read_prom(uint16_t prom[8])
{
    /*
     * MS5611-01BA datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5611-01BA
     * contains a PROM memory with 128-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * CRC field must me removed for CRC-4 calculation.
     */
    for (uint8_t i = 0; i < 8; i++) {
        prom[i] = _serial->read_16bits(CMD_MS56XX_PROM + (i << 1));
    }

    /* save the read crc */
    const uint16_t crc_read = prom[7] & 0xf;

    /* remove CRC byte */
    prom[7] &= 0xff00;

    return crc_read == crc4(prom);
}

bool AP_Baro_MS5637::_read_prom(uint16_t prom[8])
{
    /*
     * MS5637-02BA03 datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5637
     * contains a PROM memory with 112-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * 8th PROM word must be zeroed and CRC field removed for CRC-4
     * calculation.
     */
    for (uint8_t i = 0; i < 7; i++) {
        prom[i] = _serial->read_16bits(CMD_MS56XX_PROM + (i << 1));
    }

    prom[7] = 0;

    /* save the read crc */
    const uint16_t crc_read = (prom[0] & 0xf000) >> 12;

    /* remove CRC byte */
    prom[0] &= ~0xf000;

    return crc_read == crc4(prom);
}

/*
  Read the sensor. This is a state machine
  We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
  temperature does not change so quickly...
*/
void AP_Baro_MS56XX::_timer(void)
{
    // Throttle read rate to 100hz maximum.
    if (AP_HAL::micros() - _last_timer < 10000) {
        return;
    }

    if (!_serial->sem_take_nonblocking()) {
        return;
    }

    if (_state == 0) {
        // On state 0 we read temp
        uint32_t d2 = _serial->read_24bits(0);
        if (d2 != 0) {
            _s_D2 += d2;
            _d2_count++;
            if (_d2_count == 32) {
                // we have summed 32 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D2 >>= 1;
                _d2_count = 16;
            }

            if (_serial->write(ADDR_CMD_CONVERT_D1)) {      // Command to read pressure
                _state++;
            }
        } else {
            /* if read fails, re-initiate a temperature read command or we are
             * stuck */
            _serial->write(ADDR_CMD_CONVERT_D2);
        }
    } else {
        uint32_t d1 = _serial->read_24bits(0);
        if (d1 != 0) {
            // occasional zero values have been seen on the PXF
            // board. These may be SPI errors, but safest to ignore
            _s_D1 += d1;
            _d1_count++;
            if (_d1_count == 128) {
                // we have summed 128 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D1 >>= 1;
                _d1_count = 64;
            }
            // Now a new reading exists
            _updated = true;

            if (_state == 4) {
                if (_serial->write(ADDR_CMD_CONVERT_D2)) { // Command to read temperature
                    _state = 0;
                }
            } else {
                if (_serial->write(ADDR_CMD_CONVERT_D1)) { // Command to read pressure
                    _state++;
                }
            }
        } else {
            /* if read fails, re-initiate a pressure read command or we are
             * stuck */
            _serial->write(ADDR_CMD_CONVERT_D1);
        }
    }

    _last_timer = AP_HAL::micros();
    _serial->sem_give();
}

void AP_Baro_MS56XX::update()
{
    if (!_use_timer) {
        // if we're not using the timer then accumulate one more time
        // to cope with the calibration loop and minimise lag
        accumulate();
    }

    if (!_updated) {
        return;
    }
    uint32_t sD1, sD2;
    uint8_t d1count, d2count;

    // Suspend timer procs because these variables are written to
    // in "_update".
    hal.scheduler->suspend_timer_procs();
    sD1 = _s_D1; _s_D1 = 0;
    sD2 = _s_D2; _s_D2 = 0;
    d1count = _d1_count; _d1_count = 0;
    d2count = _d2_count; _d2_count = 0;
    _updated = false;
    hal.scheduler->resume_timer_procs();

    if (d1count != 0) {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0) {
        _D2 = ((float)sD2) / d2count;
    }
    _calculate();
}

/* MS5611 class */
AP_Baro_MS5611::AP_Baro_MS5611(AP_Baro &baro, AP_SerialBus *serial, bool use_timer)
    : AP_Baro_MS56XX(baro, serial, use_timer)
{
    _init();
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5611::_calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2-(((uint32_t)_c5)<<8);
    TEMP = (dT * _c6)/8388608;
    OFF = _c2 * 65536.0f + (_c4 * dT) / 128;
    SENS = _c1 * 32768.0f + (_c3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5f*Aux;
        float SENS2 = 1.25f*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    float pressure = (_D1*SENS/2097152 - OFF)/32768;
    float temperature = (TEMP + 2000) * 0.01f;
    _copy_to_frontend(_instance, pressure, temperature);
}

/* MS5607 Class */
AP_Baro_MS5607::AP_Baro_MS5607(AP_Baro &baro, AP_SerialBus *serial, bool use_timer)
    : AP_Baro_MS56XX(baro, serial, use_timer)
{
    _init();
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5607::_calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2-(((uint32_t)_c5)<<8);
    TEMP = (dT * _c6)/8388608;
    OFF = _c2 * 131072.0f + (_c4 * dT) / 64;
    SENS = _c1 * 65536.0f + (_c3 * dT) / 128;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 61.0f*Aux/16.0f;
        float SENS2 = 2.0f*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    float pressure = (_D1*SENS/2097152 - OFF)/32768;
    float temperature = (TEMP + 2000) * 0.01f;
    _copy_to_frontend(_instance, pressure, temperature);
}

/* MS563 Class */
AP_Baro_MS5637::AP_Baro_MS5637(AP_Baro &baro, AP_SerialBus *serial, bool use_timer)
    : AP_Baro_MS56XX(baro, serial, use_timer)
{
    _init();
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5637::_calculate()
{
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t raw_pressure = _D1;
    int32_t raw_temperature = _D2;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    dT = raw_temperature - (((uint32_t)_c5) << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)_c6) / 8388608;
    OFF = (int64_t)_c2 * (int64_t)131072 + ((int64_t)_c4 * (int64_t)dT) / (int64_t)64;
    SENS = (int64_t)_c1 * (int64_t)65536 + ((int64_t)_c3 * (int64_t)dT) / (int64_t)128;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT) / (int64_t)8589934592);
        int64_t aux = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = 61 * aux / 16;
        int64_t SENS2 = 29 * aux / 16;

        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    int32_t pressure = ((int64_t)raw_pressure * SENS / (int64_t)2097152 - OFF) / (int64_t)32768;
    float temperature = TEMP * 0.01f;
    _copy_to_frontend(_instance, (float)pressure, temperature);
}

/*
  Read the sensor from main code. This is only used for I2C MS5611 to
  avoid conflicts on the semaphore from calling it in a timer, which
  conflicts with the compass driver use of I2C
*/
void AP_Baro_MS56XX::accumulate(void)
{
    if (!_use_timer) {
        // the timer isn't being called as a timer, so we need to call
        // it in accumulate()
        _timer();
    }
}
