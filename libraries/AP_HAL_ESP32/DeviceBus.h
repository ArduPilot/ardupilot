#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include "Semaphores.h"
#include "AP_HAL_ESP32.h"
#include "Scheduler.h"


namespace ESP32 {

class DeviceBus {
public:
    DeviceBus(uint8_t _thread_priority);

    struct DeviceBus *next;
    Semaphore semaphore;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb, AP_HAL::Device *hal_device);
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);
    static void bus_thread(void *arg);

private:
    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;
    uint8_t thread_priority;
    void* bus_thread_handle;
    bool thread_started;
    AP_HAL::Device *hal_device;
};

}


