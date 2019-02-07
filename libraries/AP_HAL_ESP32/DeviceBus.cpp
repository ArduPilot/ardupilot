#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


using namespace ESP32;

extern const AP_HAL::HAL& hal;

DeviceBus::DeviceBus(uint8_t _thread_priority) :
    thread_priority(_thread_priority)
{
}

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
        // don't delay for less than 100usec, so one thread doesn't
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

        xTaskCreate(DeviceBus::bus_thread, name, Scheduler::DEVICE_SS,
                    this, thread_priority, &bus_thread_handle);
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
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle) {
        return false;
    }

    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);

    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    return true;
}
