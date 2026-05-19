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


 * Driver for 4 LD ... 9 LD line of pressure transducers from Keller:
 * http://www.keller-druck.com/home_e/paprod_e/4ld_e.asp
 *
 * These sensors operate on I2C and come in a variety of form factors.
 * The measurement range is between 0-200 bar depbending on the model.
 * They are definitely not the worlds smallest pressure transmitter.
 *
 * Default address is 0x40.
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_KELLERLD_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_KELLERLD_I2C_ADDR
#define HAL_BARO_KELLERLD_I2C_ADDR 0x40
#endif

class AP_Baro_KellerLD : public AP_Baro_Backend
{
public:
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

private:
    AP_Baro_KellerLD(AP_Baro &baro, AP_HAL::Device &dev);

    bool _init();

    void _timer();

    bool _read();

    void _update_and_wrap_accumulator(uint16_t pressure, uint16_t temperature, uint8_t max_count);

    AP_HAL::Device *_dev;

    /* Shared values between thread sampling the HW and main thread */
    /* These are raw outputs, not calculated values */
    struct {
        uint32_t sum_pressure;
        uint32_t sum_temperature;
        uint8_t num_samples;
    } _accum;

    uint8_t _instance;

    enum class SensorMode {
        PR_MODE = 0,    // Vented gauge
        PA_MODE = 1,    // Sealed gauge
        PAA_MODE = 2,   // Absolute
        UNDEFINED = 3,  
    };

    // to store sensor mode
    SensorMode _p_mode;
    // Model-specific offset/calibration values stored in device ROM
    //  pressure offset used in pressure calculation
    float _p_mode_offset;
    //  measurement range parameters used in pressure calculation
    float _p_min;
    float _p_max;

    // helpers for reading out calibration information:
    bool transfer_with_delays(uint8_t *send, uint8_t sendlen, uint8_t *recv, uint8_t recvlen);
    bool read_measurement_limit(float *limit, uint8_t msb_addr, uint8_t lsb_addr);
    bool read_cal();
    bool read_mode_type();
};


#endif  // AP_BARO_KELLERLD_ENABLED
