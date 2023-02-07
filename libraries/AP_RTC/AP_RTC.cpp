#include "AP_RTC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <time.h>

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

    // @Param: _TZ_MIN
    // @DisplayName: Timezone offset from UTC
    // @Description: Adds offset in +- minutes from UTC to calculate local time
    // @Range: -720 +840
    // @User: Advanced
    AP_GROUPINFO("_TZ_MIN",  2, AP_RTC, tz_min, 0),
    
    AP_GROUPEND
};

void AP_RTC::set_utc_usec(uint64_t time_utc_usec, source_type type)
{
    const uint64_t oldest_acceptable_date_us = 1640995200ULL*1000*1000; // 2022-01-01 0:00

    if (type >= rtc_source_type) {
        // e.g. system-time message when we've been set by the GPS
        return;
    }

    // check it's from an allowed sources:
    if (!(allowed_types & (1<<type))) {
        return;
    }

    // don't allow old times
    if (time_utc_usec < oldest_acceptable_date_us) {
        return;
    }

    const uint64_t now = AP_HAL::micros64();
    const int64_t tmp = int64_t(time_utc_usec) - int64_t(now);
    if (tmp < rtc_shift) {
        // can't allow time to go backwards, ever
        return;
    }
    WITH_SEMAPHORE(rsem);

    rtc_shift = tmp;

    // update hardware clock:
    if (type != SOURCE_HW) {
        hal.util->set_hw_rtc(time_utc_usec);
    }

    rtc_source_type = type;

#if HAL_GCS_ENABLED
    // update signing timestamp
    GCS_MAVLINK::update_signing_timestamp(time_utc_usec);
#endif
}

bool AP_RTC::get_utc_usec(uint64_t &usec) const
{
    if (rtc_source_type == SOURCE_NONE) {
        return false;
    }
    usec = AP_HAL::micros64() + rtc_shift;
    return true;
}

bool AP_RTC::get_system_clock_utc(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
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

bool AP_RTC::get_local_time(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
{
     // get local time of day in ms
    uint64_t time_ms = 0;
    uint64_t ms_local = 0;
    if (!get_utc_usec(time_ms)) {
        return false;
    }
    time_ms /= 1000U;
    ms_local = time_ms + (tz_min * 60000);

    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = ms_local % 1000;
    uint32_t sec_ms = (ms_local % (60 * 1000)) - ms;
    uint32_t min_ms = (ms_local % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (ms_local % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);

    return true;
}

// get milliseconds from now to a target time of day expressed as
// hour, min, sec, ms.  Match starts from first value that is not
// -1. I.e. specifying hour=-1, minutes=10 will ignore the hour and
// return time until 10 minutes past 12am (utc) NOTE: if this time has
// just past then you can expect a return value of roughly 86340000 -
// the number of milliseconds in a day.
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
    uint8_t curr_hour, curr_min, curr_sec;
    uint16_t curr_ms;
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


/*
  mktime replacement from Samba
 */
time_t AP_RTC::mktime(const struct tm *t)
{
    time_t epoch = 0;
    int n;
    int mon [] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }, y, m, i;
    const unsigned MINUTE = 60;
    const unsigned HOUR = 60*MINUTE;
    const unsigned DAY = 24*HOUR;
    const unsigned YEAR = 365*DAY;

    if (t->tm_year < 70) {
        return (time_t)-1;
    }

    n = t->tm_year + 1900 - 1;
    epoch = (t->tm_year - 70) * YEAR +
            ((n / 4 - n / 100 + n / 400) - (1969 / 4 - 1969 / 100 + 1969 / 400)) * DAY;

    y = t->tm_year + 1900;
    m = 0;

    for (i = 0; i < t->tm_mon; i++) {
        epoch += mon [m] * DAY;
        if (m == 1 && y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) {
            epoch += DAY;
        }

        if (++m > 11) {
            m = 0;
            y++;
        }
    }

    epoch += (t->tm_mday - 1) * DAY;
    epoch += t->tm_hour * HOUR + t->tm_min * MINUTE + t->tm_sec;

    return epoch;
}

// singleton instance
AP_RTC *AP_RTC::_singleton;

namespace AP {

AP_RTC &rtc()
{
    return *AP_RTC::get_singleton();
}

}
