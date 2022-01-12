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
    using AP_WheelEncoder_Backend::AP_WheelEncoder_Backend;

    // update state
    void update(void) override;

private:

    // check if pin has changed and initialise gpio event callback
    void update_pin(uint8_t &pin, uint8_t new_pin, uint8_t &pin_value);

    // gpio interrupt handlers
    void irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp);  // combined irq handler

    // convert pin a and b status to phase
    static uint8_t pin_ab_to_phase(bool pin_a, bool pin_b);

    // update phase, distance_count and error count using pin a and b's latest state
    void update_phase_and_error_count();

    struct IrqState {
        uint8_t  phase;             // current phase of encoder (from 0 to 3)
        int32_t  distance_count;    // distance measured by cumulative steps forward or backwards since last update
        uint32_t total_count;       // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;       // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;   // system time of last update from encoder
    } irq_state;

    // private members
    uint8_t last_pin_a = -1;
    uint8_t last_pin_b = -1;

    uint8_t last_pin_a_value;
    uint8_t last_pin_b_value;
};
