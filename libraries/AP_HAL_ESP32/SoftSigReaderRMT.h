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

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_HAL_ESP32.h"
#include "driver/rmt.h"

namespace ESP32
{

class SoftSigReaderRMT
{
public:
    // get singleton
    static SoftSigReaderRMT *get_instance(void)
    {
        return _instance;
    }

    void init();
    bool read(uint32_t &widths0, uint32_t &widths1);
private:
    RingbufHandle_t rb = NULL;
    static SoftSigReaderRMT *_instance;

    uint16_t last_value;
    bool started = false;

};
}

