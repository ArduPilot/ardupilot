
#include "Scheduler.h"

using namespace Empty;

EmptyScheduler::EmptyScheduler()
{}

void EmptyScheduler::init(void* machtnichts)
{}

void EmptyScheduler::delay(uint32_t ms)
{}

uint32_t EmptyScheduler::millis() {
    return 10000;
}

uint32_t EmptyScheduler::micros() {
    return 200000;
}

void EmptyScheduler::delay_microseconds(uint16_t us)
{}

void EmptyScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void EmptyScheduler::register_timer_process(AP_HAL::TimedProc k)
{}

bool EmptyScheduler::defer_timer_process(AP_HAL::TimedProc k) {
    if (k) k(5000);
    return true;
}

void EmptyScheduler::register_timer_failsafe(AP_HAL::TimedProc,
            uint32_t period_us)
{}

void EmptyScheduler::suspend_timer_procs()
{}

void EmptyScheduler::resume_timer_procs()
{}

void EmptyScheduler::begin_atomic()
{}

void EmptyScheduler::end_atomic()
{}

void EmptyScheduler::reboot() {
    for(;;);
}
