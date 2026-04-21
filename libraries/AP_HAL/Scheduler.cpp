#include "Scheduler.h"
#include "AP_HAL.h"
#include <stdio.h>

using namespace AP_HAL;

extern const AP_HAL::HAL& hal;

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
                                        uint16_t min_time_ms)
{
    _delay_cb = proc;
    _min_delay_cb_ms = min_time_ms;
}

void Scheduler::call_delay_cb()
{
    if (_delay_cb == nullptr) {
        return;
    }
    if (_in_delay_callback) {
        // don't recurse!
        return;
    }
    _in_delay_callback = true;
    _delay_cb();
    _in_delay_callback = false;
}

ExpectDelay::ExpectDelay(uint32_t ms)
{
    hal.scheduler->expect_delay_ms(ms);
}

ExpectDelay::~ExpectDelay()
{
    hal.scheduler->expect_delay_ms(0);
}

/*
  implement TimeCheck class for TIME_CHECK() support
 */
TimeCheck::TimeCheck(uint32_t _limit_ms, const char *_file, uint32_t _line) :
    limit_ms(_limit_ms),
    line(_line),
    file(_file)
{
    start_ms = AP_HAL::millis();
}

TimeCheck::~TimeCheck()
{
    const uint32_t end_ms = AP_HAL::millis();
    const uint32_t delta_ms = end_ms - start_ms;
    if (delta_ms > limit_ms) {
        ::printf("Delta %u at %s:%u\n", unsigned(delta_ms), file, unsigned(line));
    }
}
