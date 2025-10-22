#include "AP_RTC_config.h"

#if AP_RTC_ENABLED

#include "AP_RTC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/time.h>

#define DEBUG_RTC_SHIFT 0

#if DEBUG_RTC_SHIFT
#include <AP_Logger/AP_Logger.h>
#endif

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

    // only allow time to be moved forward from the same sourcetype
    // while the vehicle is disarmed:
    if (hal.util->get_soft_armed()) {
        if (type >= rtc_source_type) {
            // e.g. system-time message when we've been set by the GPS
            return;
        }
    } else {
        // vehicle is disarmed; accept (e.g.) GPS time source updates
        if (type > rtc_source_type) {
            // e.g. system-time message when we've been set by the GPS
            return;
        }
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

#if DEBUG_RTC_SHIFT
    uint64_t old_utc = 0;
    UNUSED_RESULT(get_utc_usec(old_utc));
#endif

    rtc_shift = tmp;

    // update hardware clock:
    if (type != SOURCE_HW) {
        hal.util->set_hw_rtc(time_utc_usec);
    }

    rtc_source_type = type;

#if AP_MAVLINK_SIGNING_ENABLED
    // update signing timestamp
    GCS_MAVLINK::update_signing_timestamp(time_utc_usec);
#endif  // AP_MAVLINK_SIGNING_ENABLED

#if DEBUG_RTC_SHIFT
    uint64_t new_utc = 0;
    UNUSED_RESULT(get_utc_usec(new_utc));
    if (old_utc != new_utc) {
        if (AP::logger().should_log(0xFFFF)){
            // log to AP_Logger
            // @LoggerMessage: RTC
            // @Description: Information about RTC clock resets
            // @Field: TimeUS: Time since system startup
            // @Field: old_utc: old time
            // @Field: new_utc: new time
            AP::logger().WriteStreaming(
                "RTC",
                "TimeUS,old_utc,new_utc",
                "sss",
                "FFF",
                "QQQ",
                AP_HAL::micros64(),
                old_utc,
                new_utc
                );
        }
    }
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

void AP_RTC::clock_ms_to_hms_fields(const uint64_t time_ms, uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
{
    // separate time into ms, sec, min, hour and days but all expressed in milliseconds
    ms = time_ms % 1000;
    uint32_t sec_ms = (time_ms % (60 * 1000)) - ms;
    uint32_t min_ms = (time_ms % (60 * 60 * 1000)) - sec_ms - ms;
    uint32_t hour_ms = (time_ms % (24 * 60 * 60 * 1000)) - min_ms - sec_ms - ms;

    // convert times as milliseconds into appropriate units
    sec = sec_ms / 1000;
    min = min_ms / (60 * 1000);
    hour = hour_ms / (60 * 60 * 1000);
}

bool AP_RTC::clock_s_to_date_fields(const uint32_t utc_sec32, uint16_t& year, uint8_t& month, uint8_t& day, uint8_t &hour, uint8_t &min, uint8_t &sec, uint8_t &wday) const
{
    const time_t utc_sec = utc_sec32;
    struct tm tmd {};
    struct tm* tm = gmtime_r(&utc_sec, &tmd);
    if (tm == nullptr) {
        return false;
    }
    year = tm->tm_year+1900;    /* Year, 20xx.  */
    month = tm->tm_mon;         /* Month.	[0-11] */
    day = tm->tm_mday;          /* Day.		[1-31] */
    hour = tm->tm_hour;         /* Hours.	[0-23] */
    min = tm->tm_min;           /* Minutes.	[0-59] */
    sec = tm->tm_sec;           /* Seconds.	[0-60] (1 leap second) */
    wday = tm->tm_wday;         /* week day, [0-6] */
    return true;
}

/*
  return true for leap years
 */
bool AP_RTC::_is_leap(uint32_t y)
{
    y += 1900;
    return (y % 4) == 0 && ((y % 100) != 0 || (y % 400) == 0);
}

/*
  implementation of timegm() (from Samba)
*/
uint32_t AP_RTC::_timegm(struct tm &tm)
{
    static const uint8_t ndays[2][12] = {
		{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
		{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};
    uint32_t res = 0;

    if (tm.tm_mon > 12 ||
        tm.tm_mday > 31 ||
        tm.tm_min > 60 ||
        tm.tm_sec > 60 ||
        tm.tm_hour > 24) {
		/* invalid tm structure */
		return 0;
	}
	
    for (auto i = 70; i < tm.tm_year; i++) {
        res += _is_leap(i) ? 366 : 365;
    }
	
    for (auto i = 0; i < tm.tm_mon; i++) {
        res += ndays[_is_leap(tm.tm_year)][i];
    }
    res += tm.tm_mday - 1U;
    res *= 24U;
	res += tm.tm_hour;
    res *= 60U;
	res += tm.tm_min;
    res *= 60U;
	res += tm.tm_sec;
	return res;
}

uint32_t AP_RTC::date_fields_to_clock_s(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) const
{
    struct tm tm {};
    tm.tm_year = year - 1900;
    tm.tm_mon = month;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = min;
    tm.tm_sec = sec;
    return _timegm(tm);
}

bool AP_RTC::get_system_clock_utc(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
{
    // get time of day in ms
    uint64_t time_ms;
    if (!get_utc_usec(time_ms)) {
        return false;
    }
    time_ms /= 1000U;
    clock_ms_to_hms_fields(time_ms, hour, min, sec, ms);
    return true;
}

bool AP_RTC::get_local_time(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
{
     // get local time of day in ms
    uint64_t time_ms;
    if (!get_utc_usec(time_ms)) {
        return false;
    }
    time_ms /= 1000U;
    const uint64_t ms_local = time_ms + (tz_min * 60000);
    clock_ms_to_hms_fields(ms_local, hour, min, sec, ms);
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
        return static_cast<uint32_t>(total_delay_ms + 1000);
    }

    // calculate sec to target
    if (largest_element >= 2) {
        total_delay_ms += (sec - curr_sec)*1000;
    }
    if (largest_element == 2 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms + (60*1000));
    }

    // calculate min to target
    if (largest_element >= 3) {
        total_delay_ms += (min - curr_min)*60*1000;
    }
    if (largest_element == 3 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms + (60*60*1000));
    }

    // calculate hours to target
    if (largest_element >= 4) {
        total_delay_ms += (hour - curr_hour)*60*60*1000;
    }
    if (largest_element == 4 && total_delay_ms < 0) {
        return static_cast<uint32_t>(total_delay_ms + (24*60*60*1000));
    }

    // total delay in milliseconds
    return static_cast<uint32_t>(total_delay_ms);
}

// get date and time.  Returns true on success and fills in year, month, day, hour, min, sec and ms
// year is the regular Gregorian year, month is 0~11, day is 1~31, hour is 0~23, minute is 0~59, second is 0~60 (1 leap second), ms is 0~999
bool AP_RTC::get_date_and_time_utc(uint16_t& year, uint8_t& month, uint8_t& day, uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const
{
    // get local time of day in ms
    uint64_t time_us = 0;
    if (!get_utc_usec(time_us)) {
        return false;
    }
    time_t utc_sec = time_us / (1000U * 1000U);
    struct tm tmd {};
    struct tm* tm = gmtime_r(&utc_sec, &tmd);
    if (tm == nullptr) {
        return false;
    }
    year = tm->tm_year+1900;    /* Year	- 1900.  */
    month = tm->tm_mon;         /* Month.	[0-11] */
    day = tm->tm_mday;          /* Day.		[1-31] */
    hour = tm->tm_hour;         /* Hours.	[0-23] */
    min = tm->tm_min;           /* Minutes.	[0-59] */
    sec = tm->tm_sec;           /* Seconds.	[0-60] (1 leap second) */
    ms = (time_us / 1000U) % 1000U; /* milliseconds [0-999] */
    return true;
}

// singleton instance
AP_RTC *AP_RTC::_singleton;

namespace AP {

AP_RTC &rtc()
{
    return *AP_RTC::get_singleton();
}

}

#endif  // AP_RTC_ENABLED
