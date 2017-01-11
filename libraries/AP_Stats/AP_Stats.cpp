#include "AP_Stats.h"

#include <AP_Math/AP_Math.h>

const extern AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOTCNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @User: Standard
    AP_GROUPINFO("_BOOTCNT",    0, AP_Stats, params.bootcount, 0),

    // @Param: _FLTTIME
    // @DisplayName: Total FlightTime
    // @Description: Total FlightTime (seconds)
    // @Units: seconds
    // @User: Standard
    AP_GROUPINFO("_FLTTIME",    1, AP_Stats, params.flttime, 0),

    // @Param: _RUNTIME
    // @DisplayName: Total RunTime
    // @Description: Total time autopilot has run
    // @Units: seconds
    // @User: Standard
    AP_GROUPINFO("_RUNTIME",    2, AP_Stats, params.runtime, 0),

    // @Param: _RESET
    // @DisplayName: Reset time
    // @Description: Seconds since January 1st 2016 (Unix epoch+1451606400) since reset (set to 0 to reset statistics)
    // @Units: seconds
    // @User: Standard
    AP_GROUPINFO("_RESET",    3, AP_Stats, params.reset, 1),

    AP_GROUPEND
};

void AP_Stats::copy_variables_from_parameters()
{
    flttime = params.flttime;
    runtime = params.runtime;
    reset = params.reset;
}

void AP_Stats::init()
{
    params.bootcount.set_and_save(params.bootcount+1);

    // initialise our variables from parameters:
    copy_variables_from_parameters();
}


void AP_Stats::flush()
{
    params.flttime.set_and_save(flttime);
    params.runtime.set_and_save(runtime);
}

void AP_Stats::update_flighttime()
{
    if (_flying_ms) {
        const uint32_t now = AP_HAL::millis();
        const uint32_t delta = (now - _flying_ms)/1000;
        flttime += delta;
        _flying_ms += delta*1000;
    }
}

void AP_Stats::update_runtime()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t delta = (now - _last_runtime_ms)/1000;
    runtime += delta;
    _last_runtime_ms += delta*1000;
}

void AP_Stats::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms -  last_flush_ms > flush_interval_ms) {
        update_flighttime();
        update_runtime();
        flush();
        last_flush_ms = now_ms;
    }

    const uint32_t params_reset = params.reset;
    if (params_reset != reset || params_reset == 0) {
        params.bootcount.set_and_save(params_reset == 0 ? 1 : 0);
        params.flttime.set_and_save(0);
        params.runtime.set_and_save(0);
        uint32_t system_clock = hal.util->get_system_clock_ms() / 1000;
        // can't store Unix seconds in a 32-bit float.  Change the
        // time base to Jan 1st 2016:
        system_clock -= 1451606400;
        params.reset.set_and_save(system_clock);
        copy_variables_from_parameters();
    }

}

void AP_Stats::set_flying(const bool is_flying)
{
    if (is_flying) {
        if (!_flying_ms) {
            _flying_ms = AP_HAL::millis();
        }
    } else {
        update_flighttime();
        _flying_ms = 0;
    }
}
