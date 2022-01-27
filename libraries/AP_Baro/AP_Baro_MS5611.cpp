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
#include "AP_Baro_MS5611.h"

#if AP_BARO_MS56XX_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL &hal;

static const uint8_t CMD_MS56XX_RESET = 0x1E;
static const uint8_t CMD_MS56XX_READ_ADC = 0x00;

/* PROM start address */
static const uint8_t CMD_MS56XX_PROM = 0xA0;

/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256  0x40
#define ADDR_CMD_CONVERT_D1_OSR512  0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50
#define ADDR_CMD_CONVERT_D2_OSR512  0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024;
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024;

/*
  constructor
 */
AP_Baro_MS56XX::AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
    , _ms56xx_type(ms56xx_type)
{
}

AP_Baro_Backend *AP_Baro_MS56XX::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       enum MS56XX_TYPE ms56xx_type)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_MS56XX *sensor = new AP_Baro_MS56XX(baro, std::move(dev), ms56xx_type);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_MS56XX::_init()
{
    if (!_dev) {
        return false;
    }

    _dev->get_semaphore()->take_blocking();

    // high retries for init
    _dev->set_retries(10);
    
    uint16_t prom[8];
    bool prom_read_ok = false;

    _dev->transfer(&CMD_MS56XX_RESET, 1, nullptr, 0);
    hal.scheduler->delay(4);
    
    const char *name = "MS5611";
    switch (_ms56xx_type) {
    case BARO_MS5607:
        name = "MS5607";
        FALLTHROUGH;
    case BARO_MS5611:
        prom_read_ok = _read_prom_5611(prom);
        break;
    case BARO_MS5837:
        name = "MS5837";
        prom_read_ok = _read_prom_5637(prom);
        break;
    case BARO_MS5637:
        name = "MS5637";
        prom_read_ok = _read_prom_5637(prom);
        break;
    }

    if (!prom_read_ok) {
        _dev->get_semaphore()->give();
        return false;
    }

    printf("%s found on bus %u address 0x%02x\n", name, _dev->bus_num(), _dev->get_bus_address());

    // Save factory calibration coefficients
    _cal_reg.c1 = prom[1];
    _cal_reg.c2 = prom[2];
    _cal_reg.c3 = prom[3];
    _cal_reg.c4 = prom[4];
    _cal_reg.c5 = prom[5];
    _cal_reg.c6 = prom[6];

    // Send a command to read temperature first
    _dev->transfer(&ADDR_CMD_CONVERT_TEMPERATURE, 1, nullptr, 0);
    _state = 0;

    memset(&_accum, 0, sizeof(_accum));

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_MS5611);
    set_bus_id(_instance, _dev->get_bus_id());

    if (_ms56xx_type == BARO_MS5837) {
        _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);
    }

    // lower retries for run
    _dev->set_retries(3);
    
    _dev->get_semaphore()->give();

    /* Request 100Hz update */
    _dev->register_periodic_callback(10 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Baro_MS56XX::_timer, void));
    return true;
}

uint16_t AP_Baro_MS56XX::_read_prom_word(uint8_t word)
{
    const uint8_t reg = CMD_MS56XX_PROM + (word << 1);
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, sizeof(val))) {
        return 0;
    }
    return (val[0] << 8) | val[1];
}

uint32_t AP_Baro_MS56XX::_read_adc()
{
    uint8_t val[3];
    if (!_dev->transfer(&CMD_MS56XX_READ_ADC, 1, val, sizeof(val))) {
        return 0;
    }
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

bool AP_Baro_MS56XX::_read_prom_5611(uint16_t prom[8])
{
    /*
     * MS5611-01BA datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5611-01BA
     * contains a PROM memory with 128-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * CRC field must me removed for CRC-4 calculation.
     */
    bool all_zero = true;
    for (uint8_t i = 0; i < 8; i++) {
        prom[i] = _read_prom_word(i);
        if (prom[i] != 0) {
            all_zero = false;
        }
    }

    if (all_zero) {
        return false;
    }

    /* save the read crc */
    const uint16_t crc_read = prom[7] & 0xf;

    /* remove CRC byte */
    prom[7] &= 0xff00;

    return crc_read == crc_crc4(prom);
}

bool AP_Baro_MS56XX::_read_prom_5637(uint16_t prom[8])
{
    /*
     * MS5637-02BA03 datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5637
     * contains a PROM memory with 112-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * 8th PROM word must be zeroed and CRC field removed for CRC-4
     * calculation.
     */
    bool all_zero = true;
    for (uint8_t i = 0; i < 7; i++) {
        prom[i] = _read_prom_word(i);
        if (prom[i] != 0) {
            all_zero = false;
        }
    }

    if (all_zero) {
        return false;
    }

    prom[7] = 0;

    /* save the read crc */
    const uint16_t crc_read = (prom[0] & 0xf000) >> 12;

    /* remove CRC byte */
    prom[0] &= ~0xf000;

    return crc_read == crc_crc4(prom);
}

/*
 * Read the sensor with a state machine
 * We read one time temperature (state=0) and then 4 times pressure (states 1-4)
 *
 * Temperature is used to calculate the compensated pressure and doesn't vary
 * as fast as pressure. Hence we reuse the same temperature for 4 samples of
 * pressure.
*/
void AP_Baro_MS56XX::_timer(void)
{
    uint8_t next_cmd;
    uint8_t next_state;
    uint32_t adc_val = _read_adc();

    /*
     * If read fails, re-initiate a read command for current state or we are
     * stuck
     */
    if (adc_val == 0) {
        next_state = _state;
    } else {
        next_state = (_state + 1) % 5;
    }

    next_cmd = next_state == 0 ? ADDR_CMD_CONVERT_TEMPERATURE
                               : ADDR_CMD_CONVERT_PRESSURE;
    if (!_dev->transfer(&next_cmd, 1, nullptr, 0)) {
        return;
    }

    /* if we had a failed read we are all done */
    if (adc_val == 0 || adc_val == 0xFFFFFF) {
        // a failed read can mean the next returned value will be
        // corrupt, we must discard it. This copes with MISO being
        // pulled either high or low
        _discard_next = true;
        return;
    }

    if (_discard_next) {
        _discard_next = false;
        _state = next_state;
        return;
    }

    WITH_SEMAPHORE(_sem);

    if (_state == 0) {
        _update_and_wrap_accumulator(&_accum.s_D2, adc_val,
                                     &_accum.d2_count, 32);
    } else if (pressure_ok(adc_val)) {
        _update_and_wrap_accumulator(&_accum.s_D1, adc_val,
                                     &_accum.d1_count, 128);
    }
    
    _state = next_state;
}

void AP_Baro_MS56XX::_update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                                  uint8_t *count, uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void AP_Baro_MS56XX::update()
{
    uint32_t sD1, sD2;
    uint8_t d1count, d2count;

    {
        WITH_SEMAPHORE(_sem);

        if (_accum.d1_count == 0) {
            return;
        }

        sD1 = _accum.s_D1;
        sD2 = _accum.s_D2;
        d1count = _accum.d1_count;
        d2count = _accum.d2_count;
        memset(&_accum, 0, sizeof(_accum));
    }

    if (d1count != 0) {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0) {
        _D2 = ((float)sD2) / d2count;
    }

    switch (_ms56xx_type) {
    case BARO_MS5607:
        _calculate_5607();
        break;
    case BARO_MS5611:
        _calculate_5611();
        break;
    case BARO_MS5637:
        _calculate_5637();
        break;
    case BARO_MS5837:
        _calculate_5837();
    }
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS56XX::_calculate_5611()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2-(((uint32_t)_cal_reg.c5)<<8);
    TEMP = (dT * _cal_reg.c6)/8388608;
    OFF = _cal_reg.c2 * 65536.0f + (_cal_reg.c4 * dT) / 128;
    SENS = _cal_reg.c1 * 32768.0f + (_cal_reg.c3 * dT) / 256;

    TEMP += 2000;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = sq(TEMP-2000.0);
        float OFF2 = 2.5f*Aux;
        float SENS2 = 1.25f*Aux;
        if (TEMP < -1500) {
            // extra compensation for temperatures below -15C
            OFF2 += 7 * sq(TEMP+1500);
            SENS2 += sq(TEMP+1500) * 11.0*0.5;
        }
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }


    float pressure = (_D1*SENS/2097152 - OFF)/32768;
    float temperature = TEMP * 0.01f;
    _copy_to_frontend(_instance, pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS56XX::_calculate_5607()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2-(((uint32_t)_cal_reg.c5)<<8);
    TEMP = (dT * _cal_reg.c6)/8388608;
    OFF = _cal_reg.c2 * 131072.0f + (_cal_reg.c4 * dT) / 64;
    SENS = _cal_reg.c1 * 65536.0f + (_cal_reg.c3 * dT) / 128;

    TEMP += 2000;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = sq(TEMP-2000);
        float OFF2 = 61.0f*Aux/16.0f;
        float SENS2 = 2.0f*Aux;
        if (TEMP < -1500) {
            OFF2 += 15 * sq(TEMP+1500);
            SENS2 += 8 * sq(TEMP+1500);
        }
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    float pressure = (_D1*SENS/2097152 - OFF)/32768;
    float temperature = TEMP * 0.01f;
    _copy_to_frontend(_instance, pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS56XX::_calculate_5637()
{
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t raw_pressure = _D1;
    int32_t raw_temperature = _D2;

    dT = raw_temperature - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)_cal_reg.c6) / 8388608;
    OFF = (int64_t)_cal_reg.c2 * (int64_t)131072 + ((int64_t)_cal_reg.c4 * (int64_t)dT) / (int64_t)64;
    SENS = (int64_t)_cal_reg.c1 * (int64_t)65536 + ((int64_t)_cal_reg.c3 * (int64_t)dT) / (int64_t)128;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT) / (int64_t)8589934592);
        int64_t aux = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = 61 * aux / 16;
        int64_t SENS2 = 29 * aux / 16;

        if (TEMP < -1500) {
            OFF2 += 17 * sq(TEMP+1500);
            SENS2 += 9 * sq(TEMP+1500);
        }
        
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    int32_t pressure = ((int64_t)raw_pressure * SENS / (int64_t)2097152 - OFF) / (int64_t)32768;
    float temperature = TEMP * 0.01f;
    _copy_to_frontend(_instance, (float)pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS56XX::_calculate_5837()
{
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t raw_pressure = _D1;
    int32_t raw_temperature = _D2;

    // note that MS5837 has no compensation for temperatures below -15C in the datasheet

    dT = raw_temperature - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)_cal_reg.c6) / 8388608;
    OFF = (int64_t)_cal_reg.c2 * (int64_t)65536 + ((int64_t)_cal_reg.c4 * (int64_t)dT) / (int64_t)128;
    SENS = (int64_t)_cal_reg.c1 * (int64_t)32768 + ((int64_t)_cal_reg.c3 * (int64_t)dT) / (int64_t)256;

    if (TEMP < 2000) {
        // second order temperature compensation when under 20 degrees C
        int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT) / (int64_t)8589934592);
        int64_t aux = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = 3 * aux / 2;
        int64_t SENS2 = 5 * aux / 8;

        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    int32_t pressure = ((int64_t)raw_pressure * SENS / (int64_t)2097152 - OFF) / (int64_t)8192;
    pressure = pressure * 10; // MS5837 only reports to 0.1 mbar
    float temperature = TEMP * 0.01f;

    _copy_to_frontend(_instance, (float)pressure, temperature);
}

#endif  // AP_BARO_MS56XX_ENABLED
