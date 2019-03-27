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
/*
  simple internal combustion engine simulator class
*/

#include "SIM_ICEngine.h"
#include <stdio.h>

using namespace SITL;

/*
  update engine state, returning power output from 0 to 1
 */
float ICEngine::update(const struct sitl_input &input)
{
    bool have_ignition = ignition_servo>=0;
    bool have_choke = choke_servo>=0;
    bool have_starter = starter_servo>=0;
    float throttle_demand = (input.servos[throttle_servo]-1000) * 0.001f;

    state.ignition = have_ignition?input.servos[ignition_servo]>1700:true;
    state.choke = have_choke?input.servos[choke_servo]>1700:false;
    state.starter = have_starter?input.servos[starter_servo]>1700:false;

    uint64_t now = AP_HAL::micros64();
    float dt = (now - last_update_us) * 1.0e-6f;
    float max_change = slew_rate * 0.01f * dt;
    
    if (!have_starter) {
        // always on
        last_output = throttle_demand;
        return last_output;
    }

    if (state.value != last_state.value) {
        printf("choke:%u starter:%u ignition:%u\n",
               (unsigned)state.choke,
               (unsigned)state.starter,
               (unsigned)state.ignition);
    }
    
    if (have_ignition && !state.ignition) {
        // engine is off
        if (!state.starter) {
            goto engine_off;
        }
        // give 10% when on starter alone without ignition
        last_update_us = now;
        throttle_demand = 0.1;
        goto output;
    }
    if (have_choke && state.choke && now - start_time_us > 1000*1000UL) {
        // engine is choked, only run for 1s
        goto engine_off;
    }
    if (last_output <= 0 && !state.starter) {
        // not started
        goto engine_off;
    }
    if (start_time_us == 0 && state.starter) {
        if (throttle_demand > 0.2) {
            printf("too much throttle to start: %.2f\n", throttle_demand);
        } else {
            // start the motor
            if (start_time_us == 0) {
                printf("Engine started\n");
            }
            start_time_us = now;
        }
    }
    if (start_time_us != 0 && state.starter) {
        uint32_t starter_time_us = (now - start_time_us);
        if (starter_time_us > 3000*1000UL && !overheat) {
            overheat = true;
            printf("Starter overheat\n");            
        }
    } else {
        overheat = false;
    }

output:
    if (start_time_us != 0 && throttle_demand < 0.01) {
        // even idling it gives some thrust
        throttle_demand = 0.01;
    }

    last_output = constrain_float(throttle_demand, last_output-max_change, last_output+max_change);
    last_output = constrain_float(last_output, 0, 1);
    
    last_update_us = now;
    last_state = state;
    return last_output;

engine_off:
    if (start_time_us != 0) {
        printf("Engine stopped\n");
    }
    last_update_us = AP_HAL::micros64();
    start_time_us = 0;
    last_output = 0;
    last_state = state;
    start_time_us = 0;
    return 0;
}
