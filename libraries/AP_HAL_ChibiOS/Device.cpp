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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <stdio.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"


using namespace ChibiOS;

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
  per-bus callback thread
*/
void DeviceBus::bus_thread(void *arg)
{
    struct DeviceBus *binfo = (struct DeviceBus *)arg;

    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                if (binfo->semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
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
        if (delay < 100) {
            delay = 100;
        }
        hal.scheduler->delay_microseconds(delay);
    }
    return;
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
    if (!thread_started) {
        thread_started = true;

        hal_device = _hal_device;
        // setup a name for the thread
        const uint8_t name_len = 7;
        char *name = (char *)malloc(name_len);
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, name_len, "I2C:%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, name_len, "SPI:%u",
                     hal_device->bus_num());
            break;
        default:
            break;
        }

        thread_ctx = chThdCreateFromHeap(NULL,
                         THD_WORKING_AREA_SIZE(1024),
                         name,
                         thread_priority,           /* Initial priority.    */
                         DeviceBus::bus_thread,    /* Thread function.     */
                         this);                     /* Thread parameter.    */
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
    if (chThdGetSelfX() != thread_ctx) {
        return false;
    }

    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);

    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    return true;
}

/*
  setup to use DMA-safe bouncebuffers for device transfers
 */
void DeviceBus::bouncebuffer_setup(const uint8_t *&buf_tx, uint16_t tx_len,
                                   uint8_t *&buf_rx, uint16_t rx_len)
{
    if (buf_tx && !IS_DMA_SAFE(buf_tx)) {
        if (tx_len > bounce_buffer_tx_size) {
            if (bounce_buffer_tx_size) {
                hal.util->free_type(bounce_buffer_tx, bounce_buffer_tx_size, AP_HAL::Util::MEM_DMA_SAFE);
                bounce_buffer_tx_size = 0;
            }
            bounce_buffer_tx = (uint8_t *)hal.util->malloc_type(tx_len, AP_HAL::Util::MEM_DMA_SAFE);
            if (bounce_buffer_tx == nullptr) {
                AP_HAL::panic("Out of memory for DMA TX");
            }
            bounce_buffer_tx_size = tx_len;
        }
        memcpy(bounce_buffer_tx, buf_tx, tx_len);
        buf_tx = bounce_buffer_tx;
    }

    if (buf_rx && !IS_DMA_SAFE(buf_rx)) {
        if (rx_len > bounce_buffer_rx_size) {
            if (bounce_buffer_rx_size) {
                hal.util->free_type(bounce_buffer_rx, bounce_buffer_rx_size, AP_HAL::Util::MEM_DMA_SAFE);
                bounce_buffer_rx_size = 0;
            }
            bounce_buffer_rx = (uint8_t *)hal.util->malloc_type(rx_len, AP_HAL::Util::MEM_DMA_SAFE);
            if (bounce_buffer_rx == nullptr) {
                AP_HAL::panic("Out of memory for DMA RX");
            }
            bounce_buffer_rx_size = rx_len;
        }
        buf_rx = bounce_buffer_rx;
    }
}

/*
  complete a transfer using DMA bounce buffer
 */
void DeviceBus::bouncebuffer_rx_copy(uint8_t *buf_rx, uint16_t rx_len)
{
    memcpy(buf_rx, bounce_buffer_rx, rx_len);
}
