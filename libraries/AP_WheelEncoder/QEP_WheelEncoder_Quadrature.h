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
#pragma once

#include "AP_WheelEncoder.h"
#include "WheelEncoder_Backend.h"
#include <Filter/Filter.h>
#include <AP_Math/AP_Math.h>

class QEP_WheelEncoder_Quadrature : public AP_WheelEncoder_Backend
{
public:
    // constructor
    QEP_WheelEncoder_Quadrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state);
    //void encoder_count(uint8_t instance);

    // update state
    void update(void);

private:

    struct IrqState {
        uint32_t last_gpio_a;       // gpio used for pin a
        uint32_t last_gpio_b;       // gpio used for pin b
        int32_t  phase;             // current phase of encoder (from 0 to 3)
        int32_t  distance_count;    // distance measured by cumulative steps forward or backwards
        uint32_t total_count;       // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;       // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;   // system time of last update from encoder
    };
    static struct IrqState irq_state[WHEELENCODER_MAX_INSTANCES];


};
