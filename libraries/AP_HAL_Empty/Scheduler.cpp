
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

void Scheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void Scheduler::suspend_timer_procs()
{}

void Scheduler::resume_timer_procs()
{}

bool Scheduler::in_timerprocess() {
    return false;
}

void Scheduler::system_initialized()
{}

void Scheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}
