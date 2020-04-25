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

#include "AP_Notify/AP_Notify.h"
#include "ProfiLED.h"
#include "SRV_Channel/SRV_Channel.h"

// This limit is from the dshot driver rc out groups limit, we need at least one channel for clock
#define AP_NOTIFY_ProfiLED_MAX_INSTANCES        3

#define ProfiLED_LOW    0x33
#define ProfiLED_MEDIUM 0x7F
#define ProfiLED_HIGH   0xFF
#define ProfiLED_OFF    0x00

extern const AP_HAL::HAL& hal;

ProfiLED::ProfiLED() :
    SerialLED(ProfiLED_OFF, ProfiLED_HIGH, ProfiLED_MEDIUM, ProfiLED_LOW)
{
}

uint16_t ProfiLED::init_ports()
{
    uint16_t mask = 0;
    for (uint16_t i=0; i<AP_NOTIFY_ProfiLED_MAX_INSTANCES; i++) {
        const SRV_Channel::Aux_servo_function_t fn = (SRV_Channel::Aux_servo_function_t)((uint8_t)SRV_Channel::k_ProfiLED_1 + i);
        if (!SRV_Channels::function_assigned(fn)) {
            continue;
        }
        mask |= SRV_Channels::get_output_channel_mask(fn);
    }

    if (mask == 0) {
        return 0;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return 0;
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & mask) {
            led->set_num_profiled(chan+1, (pNotify->get_led_len()));
        }
    }

    return mask;
}
