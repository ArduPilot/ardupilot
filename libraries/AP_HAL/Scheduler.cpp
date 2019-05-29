#include "Scheduler.h"
#include "AP_HAL.h"

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
