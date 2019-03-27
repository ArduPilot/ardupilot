#include "AP_RTC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_RTC::AP_RTC()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        // it's an error to get here.  But I don't want to include
        // AP_HAL here
        return;
    }
    _singleton = this;
}


// table of user settable parameters
const AP_Param::GroupInfo AP_RTC::var_info[] = {

    // @Param: _TYPES
    // @DisplayName: Allowed sources of RTC time
    // @Description: Specifies which sources of UTC time will be accepted
    // @Bitmask: 0:GPS,1:MAVLINK_SYSTEM_TIME,2:HW
    // @User: Advanced
    AP_GROUPINFO("_TYPES",  1, AP_RTC, allowed_types, 1),

    AP_GROUPEND
};

void AP_RTC::set_utc_usec(uint64_t time_utc_usec, source_type type)
{
    if (type >= rtc_source_type) {
        // e.g. system-time message when we've been set by the GPS
        return;
    }

    // check it's from an allowed sources:
    if (!(allowed_types & (1<<type))) {
        return;
    }

    const uint64_t now = AP_HAL::micros64();
    const int64_t tmp = int64_t(time_utc_usec) - int64_t(now);
    if (tmp < rtc_shift) {
        // can't allow time to go backwards, ever
        return;
    }

    rtc_shift = tmp;

    // update hardware clock:
    if (type != SOURCE_HW) {
        hal.util->set_hw_rtc(time_utc_usec);
    }

    rtc_source_type = type;

    // update signing timestamp
    GCS_MAVLINK::update_signing_timestamp(time_utc_usec);
}

bool AP_RTC::get_utc_usec(uint64_t &usec) const
{
    if (rtc_source_type == SOURCE_NONE) {
        return false;
    }
    usec = AP_HAL::micros64() + rtc_shift;
    return true;
}

bool AP_RTC::get_system_clock_utc(int32_t &hour, int32_t &min, int32_t &sec, int32_t &ms)
{
     // get time of day in ms
    uint64_t time_ms = 0;
    if (!get_utc_usec(time_ms)) {
        return false;
    }
    time_ms /= 1000U;

    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = time_ms % 1000;
    uint32_t sec_ms = (time_ms % (60 * 1000)) - ms;
    uint32_t min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);

    return true;
}

// get milliseconds from now to a target time of day expressed as hour, min, sec, ms
// match starts from first value that is not -1. I.e. specifying hour=-1, minutes=10 will ignore the hour and return time until 10 minutes past 12am (utc)
uint32_t AP_RTC::get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms)
{
    // determine highest value specified (0=none, 1=ms, 2=sec, 3=min, 4=hour)
    int8_t largest_element = 0;
    if (hour != -1) {
        largest_element = 4;
    } else if (min != -1) {
        largest_element = 3;
    } else if (sec != -1) {
        largest_element = 2;
    } else if (ms != -1) {
        largest_element = 1;
    } else {
        // exit immediately if no time specified
        return 0;
    }

    // get start_time_ms as h, m, s, ms
    int32_t curr_hour, curr_min, curr_sec, curr_ms;
    if (!get_system_clock_utc(curr_hour, curr_min, curr_sec, curr_ms)) {
        return 0;
    }
    int32_t total_delay_ms = 0;

    // calculate ms to target
    if (largest_element >= 1) {
        total_delay_ms += ms - curr_ms;
    }
    if (largest_element == 1 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += 1000);
    }

    // calculate sec to target
    if (largest_element >= 2) {
        total_delay_ms += (sec - curr_sec)*1000;
    }
    if (largest_element == 2 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*1000));
    }

    // calculate min to target
    if (largest_element >= 3) {
        total_delay_ms += (min - curr_min)*60*1000;
    }
    if (largest_element == 3 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (60*60*1000));
    }

    // calculate hours to target
    if (largest_element >= 4) {
        total_delay_ms += (hour - curr_hour)*60*60*1000;
    }
    if (largest_element == 4 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms += (24*60*60*1000));
    }

    // total delay in milliseconds
    return static_cast<uint32_t>(total_delay_ms);
}


// singleton instance
AP_RTC *AP_RTC::_singleton;

namespace AP {

AP_RTC &rtc()
{
    return *AP_RTC::get_singleton();
}

}
