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
 * Code by David "Buzz" Bussenschutt and others
 *
  */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_HAL_ESP32.h"


#if HAL_USE_EICU == TRUE

#define INPUT_CAPTURE_FREQUENCY 1000000 //capture unit in microseconds
#ifndef SOFTSIG_MAX_SIGNAL_TRANSITIONS
#define SOFTSIG_MAX_SIGNAL_TRANSITIONS 128
#endif

namespace ESP32
{

class SoftSigReaderInt
{
public:
    SoftSigReaderInt();
    ~SoftSigReaderInt();

    /* Do not allow copies */
    CLASS_NO_COPY(SoftSigReaderInt);

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

    static void _irq_handler(void * arg);

    typedef struct PACKED {
        uint16_t w0;
        uint16_t w1;
    } pulse_t;
    ObjectBuffer<pulse_t> sigbuf{SOFTSIG_MAX_SIGNAL_TRANSITIONS};
    uint16_t last_value;
};
}
#endif // HAL_USE_EICU

