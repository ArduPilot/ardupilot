#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "Scheduler.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

YUNEECScheduler::YUNEECScheduler()
{}

void YUNEECScheduler::init(void* machtnichts)
{}

void YUNEECScheduler::delay(uint16_t ms)
{}

uint32_t YUNEECScheduler::millis() {
    return 10000;
}

uint32_t YUNEECScheduler::micros() {
    return 200000;
}

void YUNEECScheduler::delay_microseconds(uint16_t us)
{}

void YUNEECScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void YUNEECScheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void YUNEECScheduler::register_io_process(AP_HAL::MemberProc k)
{}

void YUNEECScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void YUNEECScheduler::suspend_timer_procs()
{}

void YUNEECScheduler::resume_timer_procs()
{}

bool YUNEECScheduler::in_timerprocess() {
    return false;
}

void YUNEECScheduler::begin_atomic()
{}

void YUNEECScheduler::end_atomic()
{}

bool YUNEECScheduler::system_initializing() {
    return false;
}

void YUNEECScheduler::system_initialized()
{}

void YUNEECScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void YUNEECScheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}

#endif
