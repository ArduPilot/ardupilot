#include "AP_Stats.h"

#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_AHRS/AP_AHRS.h>

const extern AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOTCNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_BOOTCNT",    0, AP_Stats, params.bootcount, 0),

    // @Param: _FLTTIME
    // @DisplayName: Total FlightTime
    // @Description: Total FlightTime (seconds)
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLTTIME",    1, AP_Stats, params.flttime, 0),

    // @Param: _RUNTIME
    // @DisplayName: Total RunTime
    // @Description: Total time autopilot has run
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RUNTIME",    2, AP_Stats, params.runtime, 0),

    // @Param: _RESET
    // @DisplayName: Statistics Reset Time
    // @Description: Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics, other set values will be ignored)
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RESET",    3, AP_Stats, params.reset, 1),

    // @Param: _FLTCNT
    // @DisplayName: Total Flight Count
    // @Description: Total number of flights
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLTCNT",    4, AP_Stats, params.fltcount, 0),

    // @Param: _DISTFLWN
    // @DisplayName: Total Distance Flown
    // @Description: Estimate of total distance flown since statistics reset
    // @Units: m
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_DISTFLWN",    5, AP_Stats, params.distance_flown_m, 0),

    AP_GROUPEND
};

AP_Stats *AP_Stats::_singleton;

// constructor
AP_Stats::AP_Stats(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_Stats::copy_variables_from_parameters()
{
    flttime = params.flttime;
    runtime = params.runtime;
    reset = params.reset;
    flttime_boot = flttime;
    fltcount = params.fltcount;
    distance_flown_m = params.distance_flown_m;
}

void AP_Stats::init()
{
    params.bootcount.set_and_save(params.bootcount+1);

    // initialise our variables from parameters:
    copy_variables_from_parameters();
}

void AP_Stats::flush()
{
    params.flttime.set_and_save_ifchanged(flttime);
    params.runtime.set_and_save_ifchanged(runtime);
    params.fltcount.set_and_save_ifchanged(fltcount);
    params.distance_flown_m.set_and_save_ifchanged(distance_flown_m);
    last_flush_ms = AP_HAL::millis();
}

void AP_Stats::update_flighttime()
{
    if (_flying_ms) {
        WITH_SEMAPHORE(sem);
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
    WITH_SEMAPHORE(sem);
    const uint32_t now_ms = AP_HAL::millis();
#if AP_AHRS_ENABLED
    update_distance_flown();
#endif  // AP_AHRS_ENABLED
    if (now_ms -  last_flush_ms > flush_interval_ms) {
        update_flighttime();
        update_runtime();
        flush();
    }
    const uint32_t params_reset = params.reset;
    if (params_reset == 0) {
        // Only reset statistics if the user explicitly sets AP_STATS_RESET parameter to zero.
        // This allows users to load parameter files (in MP, MAVProxy or any other GCS) without
        // accidentally resetting the statistics, because the AP_STATS_RESET value contained in
        // the parameter file will be ignored (unless it is zero and it is usually not zero).
        // The other statistics parameters are read-only, and the GCS should be clever enough to not set those.
        params.bootcount.set_and_save_ifchanged(0);
        params.flttime.set_and_save_ifchanged(0);
        params.runtime.set_and_save_ifchanged(0);
        params.fltcount.set_and_save_ifchanged(0);
        params.distance_flown_m.set_and_save_ifchanged(0);
        uint32_t system_clock = 0; // in seconds
#if AP_RTC_ENABLED
        uint64_t rtc_clock_us;
        if (AP::rtc().get_utc_usec(rtc_clock_us)) {
            system_clock = rtc_clock_us / 1000000;
            // can't store Unix seconds in a 32-bit float.  Change the
            // time base to Jan 1st 2016:
            system_clock -= 1451606400;
        }
#endif
        params.reset.set_and_save_ifchanged(system_clock);
        copy_variables_from_parameters();
        // Reset distance tracking state
        _last_position_valid = false;
    }
}

void AP_Stats::set_flying(const bool is_flying)
{
    if (is_flying) {
        if (!_flying_ms) {
            fltcount += 1;
            _flying_ms = AP_HAL::millis();
        }
    } else {
        if (_flying_ms) {
            update_flighttime();
            update_runtime();
            update_distance_flown();
            flush();
        }
        _flying_ms = 0;
    }
}

#if AP_AHRS_ENABLED
void AP_Stats::update_distance_flown()
{
    if (!_flying_ms) {
        // Dont count unless flying
        _last_position_valid = false;
        return;
    }

    // Run at 1Hz to reduce how much GPS variance is counted as distance
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_distance_update_ms < 1000) {
        return;
    }
    _last_distance_update_ms = now_ms;

    const AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.healthy()) {
        _last_position_valid = false;
        return;
    }

    Location current_loc;
    if (!ahrs.get_location(current_loc)) {
        _last_position_valid = false;
        return;
    }

    if (_last_position_valid) {
        float delta_m = _last_position.get_distance_NED(current_loc).length();

        // Avoid counting small distances that are probably error
        if (delta_m >= 0.5) {
            // Dont blindly add large distances that are likely just gross error / spoofing
            // Assumes vehicle traveling below mach ~0.9
            if (delta_m > 300.0f) {
                delta_m = 300.0f;
            }
            distance_flown_m += delta_m;
        }
    }

    _last_position = current_loc;
    _last_position_valid = true;
}
#endif  // AP_AHRS_ENABLED

/*
  get time in flight since boot
 */
uint32_t AP_Stats::get_flight_time_s(void)
{
    update_flighttime();
    return flttime - flttime_boot;
}

AP_Stats *AP::stats(void)
{
    return AP_Stats::get_singleton();
}
