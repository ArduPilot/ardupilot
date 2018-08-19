
#include "Scheduler.h"

#include <stdarg.h>

using namespace Empty;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{}

void Scheduler::init()
{}

void Scheduler::delay(uint16_t ms)
{}

void Scheduler::delay_microseconds(uint16_t us)
{}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void Scheduler::system_initialized()
{}

void Scheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}
