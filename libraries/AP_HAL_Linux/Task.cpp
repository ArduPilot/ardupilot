#include "Task.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace Linux {

bool Task::_run()
{
    while (true) {
        const uint32_t delay_us = task.update();
        hal.scheduler->delay_microseconds(delay_us);
    }

    return true;
}

}
