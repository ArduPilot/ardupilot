/*
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
 * Code by David "Buzz" Bussenschutt. Derived from other HAL work by Andrew Tridgell 
  */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_HAL_ESP32.h"



// hack
//#define EICUDriver int
//#define eicuchannel_t int
//#define EICUConfig int
//#define EICUChannelConfig int
//#define EICU_CHANNEL_1 1
//#define EICU_CHANNEL_2 2
//#define EICU_CHANNEL_3 3
//#define EICU_CHANNEL_4 4
//#define EICU_CHANNEL_ENUM_END 5

#if HAL_USE_EICU == TRUE

#define INPUT_CAPTURE_FREQUENCY 1000000 //capture unit in microseconds
#ifndef SOFTSIG_MAX_SIGNAL_TRANSITIONS
#define SOFTSIG_MAX_SIGNAL_TRANSITIONS 128
#endif


class ESP32::SoftSigReaderInt  {
public:
    SoftSigReaderInt();
    /* Do not allow copies */
    SoftSigReaderInt(const SoftSigReaderInt &other) = delete;
    SoftSigReaderInt &operator=(const SoftSigReaderInt&) = delete;

    // get singleton
    static SoftSigReaderInt *get_instance(void)
    {
        return _instance;
    }

    void init();
    bool read(uint32_t &widths0, uint32_t &widths1);
private:
    // singleton
    static SoftSigReaderInt *_instance;

    static void _irq_handler();
    
    //static eicuchannel_t get_pair_channel(eicuchannel_t channel);
    typedef struct PACKED {
        uint16_t w0;
        uint16_t w1;
    } pulse_t;
    ObjectBuffer<pulse_t> sigbuf{SOFTSIG_MAX_SIGNAL_TRANSITIONS};
   // EICUConfig icucfg;
   // EICUChannelConfig channel_config;
   // EICUChannelConfig aux_channel_config;
   // EICUDriver* _icu_drv = nullptr;
    uint16_t last_value;
};

#endif // HAL_USE_EICU

