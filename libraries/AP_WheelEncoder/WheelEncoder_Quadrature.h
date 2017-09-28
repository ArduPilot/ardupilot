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

class AP_WheelEncoder_Quadrature : public AP_WheelEncoder_Backend
{
public:
    // constructor
    AP_WheelEncoder_Quadrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state);

    // update state
    void update(void);

private:

    // gpio interrupt handlers
    static int irq_handler0_pina(int irq, void *context);   // instance 0's pin_a handler
    static int irq_handler0_pinb(int irq, void *context);   // instance 0's pin_b handler
    static int irq_handler1_pina(int irq, void *context);   // instance 1's pin_a handler
    static int irq_handler1_pinb(int irq, void *context);   // instance 1's pin_b handler
    static void irq_handler(uint8_t instance, bool pin_a);  // combined irq handler

    // get gpio id from pin number
    static uint32_t get_gpio(uint8_t pin_number);

    // convert pin a and b status to phase
    static uint8_t pin_ab_to_phase(bool pin_a, bool pin_b);

    // update phase, distance_count and error count using pin a and b's latest state
    static void update_phase_and_error_count(bool pin_a_now, bool pin_b_now, uint8_t &phase, int32_t &distance_count, uint32_t &total_count, uint32_t &error_count);

    struct IrqState {
        uint32_t last_gpio_a;       // gpio used for pin a
        uint32_t last_gpio_b;       // gpio used for pin b
        uint8_t  phase;             // current phase of encoder (from 0 to 3)
        int32_t  distance_count;    // distance measured by cumulative steps forward or backwards
        uint32_t total_count;       // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;       // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;   // system time of last update from encoder
    };
    static struct IrqState irq_state[WHEELENCODER_MAX_INSTANCES];

    // private members
    uint8_t last_pin_a;
    uint8_t last_pin_b;
};
