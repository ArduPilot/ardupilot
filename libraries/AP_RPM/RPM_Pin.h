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

#include "AP_RPM.h"

#include "RPM_Backend.h"
#include <Filter/Filter.h>
#include <AP_Math/AP_Math.h>

class AP_RPM_Pin : public AP_RPM_Backend
{
public:
    // constructor
    AP_RPM_Pin(AP_RPM &ranger, uint8_t instance, AP_RPM::RPM_State &_state);

    // update state
    void update(void);

private:
    static void irq_handler(uint8_t instance);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    static int irq_handler0(int irq, void *context);
    static int irq_handler1(int irq, void *context);
#else
    static void irq_handler0(void);
    static void irq_handler1(void);
#endif
    
    ModeFilterFloat_Size5 signal_quality_filter {3};
    uint8_t last_pin = -1;
    uint32_t last_gpio;
    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t dt_sum;
        uint32_t dt_count;
    };
    static struct IrqState irq_state[RPM_MAX_INSTANCES];
};
