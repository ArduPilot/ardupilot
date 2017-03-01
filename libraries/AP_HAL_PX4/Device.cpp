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
 */
#include "Device.h"

#include <arch/board/board.h>
#include "board_config.h"
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Scheduler.h"
#include "Semaphores.h"

extern bool _px4_thread_should_exit;

namespace PX4 {

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
  per-bus callback thread
*/
void *DeviceBus::bus_thread(void *arg)
{
    struct DeviceBus *binfo = (struct DeviceBus *)arg;

    // setup a name for the thread
    char name[] = "XXX:X";
    switch (binfo->hal_device->bus_type()) {
    case AP_HAL::Device::BUS_TYPE_I2C:
        snprintf(name, sizeof(name), "I2C:%u",
                 binfo->hal_device->bus_num());
        break;

    case AP_HAL::Device::BUS_TYPE_SPI:
        snprintf(name, sizeof(name), "SPI:%u",
                 binfo->hal_device->bus_num());
        break;
    default:
        break;
    }
    pthread_setname_np(pthread_self(), name);
    
    while (!_px4_thread_should_exit) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                if (binfo->semaphore.take(0)) {
                    callback->cb();
                    binfo->semaphore.give();
                }
            }
        }

        // work out when next loop is needed
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // delay for at most 50ms, to handle newly added callbacks
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // don't delay for less than 400usec, so one thread doesn't
        // completely dominate the CPU
        if (delay < 400) {
            delay = 400;
        }
        hal.scheduler->delay_microseconds(delay);
    }
    return nullptr;
}
    
AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
    if (!thread_started) {
        thread_started = true;
    
        pthread_attr_t thread_attr;
        struct sched_param param;
    
        pthread_attr_init(&thread_attr);
        pthread_attr_setstacksize(&thread_attr, 1024);
    
        param.sched_priority = thread_priority;
        (void)pthread_attr_setschedparam(&thread_attr, &param);
        pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);

        hal_device = _hal_device;
        
        pthread_create(&thread_ctx, &thread_attr, &DeviceBus::bus_thread, this);
    }
    DeviceBus::callback_info *callback = new DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;

    return callback;
}

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (!pthread_equal(pthread_self(), thread_ctx)) {
        fprintf(stderr, "can't adjust timer from unknown thread context\n");
        return false;
    }

    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);

    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    return true;
}

}
