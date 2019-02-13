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
  backend driver for airspeed from a I2C MS5525D0 sensor
 */
#include "AP_Airspeed_MS5525.h"

#include <stdio.h>
#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define MS5525D0_I2C_ADDR_1 0x76
#define MS5525D0_I2C_ADDR_2 0x77

#define REG_RESET               0x1E
#define REG_CONVERT_D1_OSR_256  0x40
#define REG_CONVERT_D1_OSR_512  0x42
#define REG_CONVERT_D1_OSR_1024 0x44
#define REG_CONVERT_D1_OSR_2048 0x46
#define REG_CONVERT_D1_OSR_4096 0x48
#define REG_CONVERT_D2_OSR_256  0x50
#define REG_CONVERT_D2_OSR_512  0x52
#define REG_CONVERT_D2_OSR_1024 0x54
#define REG_CONVERT_D2_OSR_2048 0x56
#define REG_CONVERT_D2_OSR_4096 0x58
#define REG_ADC_READ            0x00
#define REG_PROM_BASE           0xA0

// go for 1024 oversampling. This should be fast enough to reduce
// noise but low enough to keep self-heating small
#define REG_CONVERT_PRESSURE    REG_CONVERT_D1_OSR_1024
#define REG_CONVERT_TEMPERATURE REG_CONVERT_D2_OSR_1024

AP_Airspeed_MS5525::AP_Airspeed_MS5525(AP_Airspeed &_frontend, uint8_t _instance, MS5525_ADDR address) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    _address = address;
}

// probe and initialise the sensor
bool AP_Airspeed_MS5525::init()
{
    const uint8_t addresses[] = { MS5525D0_I2C_ADDR_1, MS5525D0_I2C_ADDR_2 };
    bool found = false;
    for (uint8_t i=0; i<ARRAY_SIZE(addresses); i++) {
        if (_address != MS5525_ADDR_AUTO && i != (uint8_t)_address) {
            continue;
        }
        dev = hal.i2c_mgr->get_device(get_bus(), addresses[i]);
        if (!dev) {
            continue;
        }
        if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            continue;
        }

        // lots of retries during probe
        dev->set_retries(5);

        found = read_prom();
        
        if (found) {
            printf("MS5525: Found sensor on bus %u address 0x%02x\n", get_bus(), addresses[i]);
            break;
        }
        dev->get_semaphore()->give();
    }
    if (!found) {
        printf("MS5525: no sensor found\n");
        return false;
    }

    // Send a command to read temperature first
    uint8_t reg = REG_CONVERT_TEMPERATURE;
    dev->transfer(&reg, 1, nullptr, 0);
    state = 0;
    command_send_us = AP_HAL::micros();

    // drop to 2 retries for runtime
    dev->set_retries(2);

    dev->get_semaphore()->give();

    // read at 80Hz
    dev->register_periodic_callback(1000000UL/80U,
                                    FUNCTOR_BIND_MEMBER(&AP_Airspeed_MS5525::timer, void));
    return true;
}


/**
 * CRC used by MS pressure devices
 */
uint16_t AP_Airspeed_MS5525::crc4_prom(void)
{
    uint16_t n_rem = 0;
    uint8_t n_bit;

    for (uint8_t cnt = 0; cnt < sizeof(prom); cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((prom[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(prom[cnt >> 1] >> 8);
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

bool AP_Airspeed_MS5525::read_prom(void)
{
    // reset the chip to ensure it has correct prom values loaded
    uint8_t reg = REG_RESET;
    if (!dev->transfer(&reg, 1, nullptr, 0)) {
        return false;
    }
    hal.scheduler->delay(5);

    bool all_zero = true;
    for (uint8_t i = 0; i < 8; i++) {
        be16_t val;
        if (!dev->read_registers(REG_PROM_BASE+i*2, (uint8_t *) &val,
                                 sizeof(uint16_t))) {
            return false;
        }
        prom[i] = be16toh(val);
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

    uint16_t crc_calc = crc4_prom();
    if (crc_read != crc_calc) {
        printf("MS5525: CRC mismatch 0x%04x 0x%04x\n", crc_read, crc_calc);
    }
    return crc_read == crc_calc;
}


/*
  read from the ADC
 */
int32_t AP_Airspeed_MS5525::read_adc()
{
    uint8_t val[3];
    if (!dev->read_registers(REG_ADC_READ, val, 3)) {
        return 0;
    }
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

/*
  calculate pressure and temperature
 */
void AP_Airspeed_MS5525::calculate(void)
{
    // table for the 001DS part, 1PSI range
    const uint8_t Q1 = 15;
    const uint8_t Q2 = 17;
    const uint8_t Q3 = 7;
    const uint8_t Q4 = 5;
    const uint8_t Q5 = 7;
    const uint8_t Q6 = 21;

    int64_t dT = D2 - int64_t(prom[5]) * (1UL<<Q5);
    int64_t TEMP = 2000 + (dT*int64_t(prom[6]))/(1UL<<Q6);
    int64_t OFF =  int64_t(prom[2])*(1UL<<Q2) + (int64_t(prom[4])*dT)/(1UL<<Q4);
    int64_t SENS = int64_t(prom[1])*(1UL<<Q1) + (int64_t(prom[3])*dT)/(1UL<<Q3);
    int64_t P = (D1*SENS/(1UL<<21)-OFF)/(1UL<<15);
    const float PSI_to_Pa = 6894.757f;
    float P_Pa = PSI_to_Pa * 1.0e-4 * P;
    float Temp_C = TEMP * 0.01;

#if 0
    static uint16_t counter;
    if (counter++ == 100) {
        printf("P=%.6f T=%.2f D1=%d D2=%d\n", P_Pa, Temp_C, D1, D2);
        counter=0;
    }
#endif
    
    WITH_SEMAPHORE(sem);

    pressure_sum += P_Pa;
    temperature_sum += Temp_C;
    press_count++;
    temp_count++;
    last_sample_time_ms = AP_HAL::millis();
}

// 80Hz timer
void AP_Airspeed_MS5525::timer()
{
    if (AP_HAL::micros() - command_send_us < 10000) {
        // we should avoid trying to read the ADC too soon after
        // sending the command
        return;
    }
    
    uint32_t adc_val = read_adc();

    if (adc_val == 0) {
        // we have either done a read too soon after sending the
        // request or we have tried to read the same sample twice. We
        // re-send the command now as we don't know what state the
        // sensor is in, and we do want to trigger it sending a value,
        // which we will discard
        if (dev->transfer(&cmd_sent, 1, nullptr, 0)) {
            command_send_us = AP_HAL::micros();
        }
        // when we get adc_val == 0 then then both the current value and
        // the next value we read from the sensor are invalid
        ignore_next = true;
        return;
    }
    
    /*
     * If read fails, re-initiate a read command for current state or we are
     * stuck
     */
    if (!ignore_next) {
        if (cmd_sent == REG_CONVERT_TEMPERATURE) {
            D2 = adc_val;
        } else if (cmd_sent == REG_CONVERT_PRESSURE) {
            D1 = adc_val;
            calculate();
        }
    }

    ignore_next = false;
    
    cmd_sent = (state == 0) ? REG_CONVERT_TEMPERATURE : REG_CONVERT_PRESSURE;
    if (!dev->transfer(&cmd_sent, 1, nullptr, 0)) {
        // we don't know for sure what state the sensor is in when we
        // fail to send the command, so ignore the next response
        ignore_next = true;
        return;
    }
    command_send_us = AP_HAL::micros();

    state = (state + 1) % 5;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_MS5525::get_differential_pressure(float &_pressure)
{
    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }
    _pressure = pressure;

    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_MS5525::get_temperature(float &_temperature)
{
    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    if (temp_count > 0) {
        temperature = temperature_sum / temp_count;
        temp_count = 0;
        temperature_sum = 0;
    }

    _temperature = temperature;
    return true;
}
