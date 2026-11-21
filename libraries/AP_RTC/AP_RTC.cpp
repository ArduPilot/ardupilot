#include "AP_RTC_config.h"

#if AP_RTC_ENABLED

#include "AP_RTC.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/time.h>
#include <AP_Logger/AP_Logger.h>

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

// GPS time error above which a disarmed clock is stepped forward rather than slewed
static constexpr float RTC_STEP_THRESHOLD_S = 0.05;
// largest clock rate trim, kept well above crystal tolerances
static constexpr float RTC_RATE_MAX = 500.0e-6;
// time constants of the rate trim's proportional and integral terms
static constexpr float RTC_SLEW_TC_S = 30.0;
static constexpr float RTC_DRIFT_TC_S = 300.0;
// the full rate trim lasts this long after a GPS sample, covering GPS rates down to 1Hz
static constexpr uint64_t RTC_TRIM_HOLD_US = 2000000;
// gps_locked() requirements
static constexpr float RTC_LOCK_ERR_S = 0.002;
static constexpr uint8_t RTC_LOCK_SAMPLES = 10;
// the lock is held this long after the last GPS sample; the clock runs on
// its learned rate trim meanwhile, so it stays within a few milliseconds
static constexpr uint64_t RTC_LOCK_TIMEOUT_US = 600000000;
// interval at which a slewing clock updates the hardware RTC and signing timestamp
static constexpr uint64_t RTC_SLEW_SIDE_EFFECTS_INTERVAL_US = 60000000;

void AP_RTC::set_utc_usec(uint64_t time_utc_usec, source_type type)
{
    set_utc_usec_at(time_utc_usec, type, AP_HAL::micros64());
}

void AP_RTC::set_utc_usec_at(uint64_t time_utc_usec, source_type type, uint64_t local_us)
{
    const uint64_t oldest_acceptable_date_us = 1640995200ULL*1000*1000; // 2022-01-01 0:00

    // check it's from an allowed sources:
    if (!(allowed_types & (1<<type))) {
        return;
    }

    // don't allow old times
    if (time_utc_usec < oldest_acceptable_date_us) {
        return;
    }

    bool changed = false;   // stepped, or changed source
    bool slew_side_effects = false;
    uint64_t utc_now = time_utc_usec;
#if AP_RTC_LOGGING_ENABLED
    bool log_slew = false;
    float log_err_s = 0;
    float log_rate = 0;
    float log_drift = 0;
    bool log_locked = false;
#endif

    {
        WITH_SEMAPHORE(rsem);

        // another thread may have re-anchored the clock after local_us was read
        if (rtc_source_type != SOURCE_NONE && local_us < anchor_local_us) {
            local_us = anchor_local_us;
        }

        const bool armed = hal.util->get_soft_armed();

        // a GPS time always trims a clock already set by the GPS, armed or
        // disarmed, because slewing never makes the clock jump
        const bool slewing_source = (type == SOURCE_GPS) && (rtc_source_type == SOURCE_GPS);

        // only allow time to be moved forward from the same sourcetype
        // while the vehicle is disarmed:
        if (!slewing_source) {
            if (armed) {
                if (type >= rtc_source_type) {
                    // e.g. system-time message when we've been set by the GPS
                    return;
                }
            } else {
                // vehicle is disarmed; accept (e.g.) HW time source updates
                if (type > rtc_source_type) {
                    // e.g. system-time message when we've been set by the GPS
                    return;
                }
            }
        }

        const bool have_time = (rtc_source_type != SOURCE_NONE);
        const uint64_t clock_utc = have_time ? utc_at(local_us) : 0;
        const bool forward = !have_time || (time_utc_usec >= clock_utc);

        if (slewing_source) {
            if (!armed && (time_utc_usec >= clock_utc + uint64_t(RTC_STEP_THRESHOLD_S * 1.0e6))) {
                step_to(time_utc_usec, local_us);
                changed = true;
            } else {
                slew_towards(time_utc_usec, local_us);
            }
        } else if (forward) {
            step_to(time_utc_usec, local_us);
            rtc_source_type = type;
            changed = true;
        } else if (type == SOURCE_GPS) {
            // the GPS is better than the source that set this clock, but the
            // clock is ahead of it.  Time can't go backwards, so slew to it
            rtc_source_type = SOURCE_GPS;
            gps_samples = 0;
            last_gps_local_us = local_us;
            slew_towards(time_utc_usec, local_us);
            changed = true;
            utc_now = clock_utc;
        } else {
            // can't allow time to go backwards, ever
            return;
        }

        if (!changed) {
            utc_now = utc_at(local_us);
            if (local_us - last_slew_side_effects_us >= RTC_SLEW_SIDE_EFFECTS_INTERVAL_US) {
                last_slew_side_effects_us = local_us;
                slew_side_effects = true;
            }
#if AP_RTC_LOGGING_ENABLED
            if (local_us - last_slew_log_us >= 1000000) {
                last_slew_log_us = local_us;
                log_slew = true;
                log_err_s = err_filt_s;
                log_rate = rate;
                log_drift = drift;
                log_locked = gps_locked_at(local_us);
            }
#endif
        } else {
            last_slew_side_effects_us = local_us;
        }
    }

    // the logger and storage take their own locks, so these run outside rsem

#if AP_RTC_LOGGING_ENABLED
    if (changed) {
        AP::logger().Write_RTC();
    }
    if (log_slew) {
        // @LoggerMessage: RTCD
        // @Description: RTC rate trim that keeps the clock following GPS time
        // @Field: TimeUS: Time since system startup
        // @Field: Err: filtered GPS time minus RTC time
        // @Field: Rate: RTC rate trim
        // @Field: Drift: integrated rate trim, the estimated local crystal error
        // @Field: Lck: true if the RTC is following GPS time closely
        AP::logger().WriteStreaming("RTCD", "TimeUS,Err,Rate,Drift,Lck",
                                    "ss---", "F0---", "QfffB",
                                    local_us,
                                    double(log_err_s),
                                    double(log_rate * 1.0e6),
                                    double(log_drift * 1.0e6),
                                    uint8_t(log_locked));
    }
#endif  // AP_RTC_LOGGING_ENABLED

    if (!changed && !slew_side_effects) {
        return;
    }

    // update hardware clock:
    if (type != SOURCE_HW) {
        hal.util->set_hw_rtc(utc_now);
    }

#if AP_MAVLINK_SIGNING_ENABLED
    // update signing timestamp
    GCS_MAVLINK::update_signing_timestamp(utc_now);
#endif  // AP_MAVLINK_SIGNING_ENABLED
}

// clock reading at a local time; rsem must be held
uint64_t AP_RTC::utc_at(uint64_t local_us) const
{
    // the full trim, which corrects the current error, only applies until the
    // next GPS sample is due.  After that the clock runs on the drift estimate,
    // so a GPS outage doesn't carry the last sample's noise forward
    const int64_t dt_us = int64_t(local_us - anchor_local_us);
    const int64_t trim_us = MIN(dt_us, int64_t(RTC_TRIM_HOLD_US));
    return anchor_utc_us + dt_us + int64_t(float(trim_us) * rate + float(dt_us - trim_us) * drift);
}

// jump the clock to a time; rsem must be held
void AP_RTC::step_to(uint64_t time_utc_usec, uint64_t local_us)
{
    anchor_utc_us = time_utc_usec;
    anchor_local_us = local_us;
    rate = drift;
    err_filt_s = 0;
    gps_samples = 0;
    last_gps_local_us = local_us;
}

// trim the clock rate towards a GPS time; rsem must be held
void AP_RTC::slew_towards(uint64_t time_utc_usec, uint64_t local_us)
{
    // re-anchor at the current reading so the rate change is continuous
    const uint64_t clock_utc = utc_at(local_us);
    anchor_utc_us = clock_utc;
    anchor_local_us = local_us;

    const float err_s = int64_t(time_utc_usec - clock_utc) * 1.0e-6;
    if (fabsf(err_s) < RTC_STEP_THRESHOLD_S) {
        // a large error saturates the trim, so only a small one updates the drift estimate
        const float dt_s = constrain_float((local_us - last_gps_local_us) * 1.0e-6, 0.0, 10.0);
        drift = constrain_float(drift + err_s * dt_s / (RTC_SLEW_TC_S * RTC_DRIFT_TC_S), -RTC_RATE_MAX, RTC_RATE_MAX);
        err_filt_s += (err_s - err_filt_s) * 0.1;
    } else {
        err_filt_s = err_s;
    }
    rate = constrain_float(drift + err_s / RTC_SLEW_TC_S, -RTC_RATE_MAX, RTC_RATE_MAX);

    if (gps_samples < UINT8_MAX) {
        gps_samples++;
    }
    last_gps_local_us = local_us;
}

bool AP_RTC::get_utc_usec(uint64_t &usec) const
{
    uint64_t local_us;
    return get_utc_and_local_usec(usec, local_us);
}

bool AP_RTC::get_utc_and_local_usec(uint64_t &utc_usec, uint64_t &local_usec) const
{
    WITH_SEMAPHORE(rsem);
    if (rtc_source_type == SOURCE_NONE) {
        return false;
    }
    local_usec = AP_HAL::micros64();
    utc_usec = utc_at(local_usec);
    return true;
}

bool AP_RTC::get_utc_usec_at(uint64_t local_us, uint64_t &usec) const
{
    WITH_SEMAPHORE(rsem);
    if (rtc_source_type == SOURCE_NONE) {
        return false;
    }
    usec = utc_at(local_us);
    return true;
}

bool AP_RTC::gps_locked() const
{
    // read the time under the lock so it can't predate last_gps_local_us
    WITH_SEMAPHORE(rsem);
    return gps_locked_at(AP_HAL::micros64());
}

bool AP_RTC::gps_locked_at(uint64_t local_us) const
{
    WITH_SEMAPHORE(rsem);
    return (rtc_source_type == SOURCE_GPS) &&
           (gps_samples >= RTC_LOCK_SAMPLES) &&
           (fabsf(err_filt_s) < RTC_LOCK_ERR_S) &&
           (local_us - last_gps_local_us < RTC_LOCK_TIMEOUT_US);
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

    if (tm.tm_mon > 12 ||
        tm.tm_mday > 31 ||
        tm.tm_min > 60 ||
        tm.tm_sec > 60 ||
        tm.tm_hour > 24 ||
        tm.tm_year < 70) {
		/* invalid tm structure, or pre-1970 date (matches old loop's implicit floor) */
		return 0;
	}
	
    // closed-form day count instead of a per-year _is_leap loop (same identity glibc/libc++ use for civil-to-days)
    const uint32_t calendar_year = tm.tm_year + 1900U;
    uint32_t res = (tm.tm_year - 70U) * 365U
        + (calendar_year - 1U) / 4U   - 492U
        - (calendar_year - 1U) / 100U + 19U
        + (calendar_year - 1U) / 400U - 4U;

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
