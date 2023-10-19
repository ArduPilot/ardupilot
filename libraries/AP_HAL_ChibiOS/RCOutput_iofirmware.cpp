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
 * Code by Andy Piper and Siddharth Bharat Purohit
 * 
 * There really is no dshot reference. For information try these resources:
 * https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/
 * https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
 */

#include <hal.h>

#if defined(IOMCU_FW) && HAL_DSHOT_ENABLED
// need to give the little guy as much help as possible
#pragma GCC optimize("O2")

#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "GPIO.h"
#include "Scheduler.h"

#if HAL_USE_PWM == TRUE

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

THD_WORKING_AREA(dshot_thread_wa, 64);
void RCOutput::timer_tick()
{
    if (dshot_timer_setup) {
        return;
    }

    bool dshot_enabled = false;
    for (uint8_t i = 0; i < NUM_GROUPS; i++ ) {
        pwm_group &group = pwm_group_list[i];
        if (is_dshot_protocol(group.current_mode)) {
            dshot_enabled = true;
            break;
        }
    }
    if (!dshot_timer_setup && dshot_enabled) {
        chThdCreateStatic(dshot_thread_wa, sizeof(dshot_thread_wa),
                            APM_RCOUT_PRIORITY, &RCOutput::dshot_send_trampoline, this);
        dshot_timer_setup = true;
    }
}

void RCOutput::dshot_send_trampoline(void *p)
{
    RCOutput *rcout = (RCOutput *)p;
    rcout->rcout_thread();
}

/*
  thread for handling RCOutput send on IOMCU
 */
void RCOutput::rcout_thread() {
    // don't start outputting until fully configured
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    rcout_thread_ctx = chThdGetSelfX();

    while (true) {
        chEvtWaitOne(EVT_PWM_SEND | EVT_PWM_SYNTHETIC_SEND);

        // this is when the cycle is supposed to start
        if (_dshot_cycle == 0) {
            // register a timer for the next tick if push() will not be providing it
            if (_dshot_rate != 1) {
                chVTSet(&_dshot_rate_timer, chTimeUS2I(_dshot_period_us), dshot_update_tick, this);
            }
        }

        // main thread requested a new dshot send or we timed out - if we are not running
        // as a multiple of loop rate then ignore EVT_PWM_SEND events to preserve periodicity
        dshot_send_groups(0);
#if AP_HAL_SHARED_DMA_ENABLED
        dshot_collect_dma_locks(0);
#endif
        if (_dshot_rate > 0) {
            _dshot_cycle = (_dshot_cycle + 1) % _dshot_rate;
        }
    }
}

#endif // HAL_USE_PWM

#endif // IOMCU_FW && HAL_DSHOT_ENABLED
