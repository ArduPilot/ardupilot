#include "AP_Stats.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOTCNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @User: Standard
    AP_GROUPINFO("_BOOTCNT",    0, AP_Stats, params.bootcount, 0),

    AP_GROUPEND
};

void AP_Stats::init()
{
    params.bootcount.set_and_save(params.bootcount+1);
}

void AP_Stats::flush()
{
}

void AP_Stats::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms -  last_flush_ms > flush_interval_ms) {
        flush();
        last_flush_ms = now_ms;
    }
}
