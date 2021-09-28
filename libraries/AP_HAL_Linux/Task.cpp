#include "Task.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

namespace Linux {

bool Task::_run()
{
    if (!_task_init) {
        return false;
    }
    if (!_task_body) {
        return false;
    }

    _task_init();
    while (true) {
        const uint32_t delay_us = _task_body();
        hal.scheduler->delay_microseconds(delay_us);
    }

    return true;
}

}
