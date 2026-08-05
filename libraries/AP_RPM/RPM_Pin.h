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

#include "AP_RPM_config.h"

#if AP_RPM_PIN_ENABLED

#include "RPM_Backend.h"

#include <Filter/Filter.h>
#define RPM_PIN_BUFFER_SIZE 16 //must be power of 2 for buffer to work correctly


class AP_RPM_Pin : public AP_RPM_Backend
{
public:

    using AP_RPM_Backend::AP_RPM_Backend;

    // update state
    void update(void) override;

private:

    HampelFilter<float, 5> signal_quality_filter;
    int8_t last_pin = -1;       // last pin number checked vs PIN parameter
    bool interrupt_attached;    // true if an interrupt has been attached to last_pin
    struct IrqState {
        uint32_t last_pulse_us;
        uint32_t buffer[RPM_PIN_BUFFER_SIZE];
        uint8_t last_sample;
    };
    static struct IrqState irq_state[RPM_MAX_INSTANCES];
    uint8_t last_sample_used;

    void irq_handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);

};

#endif  // AP_RPM_PIN_ENABLED
