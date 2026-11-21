#pragma once

#include "AP_RTC_config.h"

#if AP_RTC_ENABLED

#include <AP_Param/AP_Param.h>
#include "AP_RTC_RemoteClocks.h"

#include <stdint.h>

class AP_RTC {

public:

    AP_RTC();

    static const struct AP_Param::GroupInfo var_info[];

    AP_UInt8 allowed_types;
    AP_Int16 tz_min;

    // ordering is important in source_type; lower-numbered is
    // considered a better time source.  These values are documented
    // and used in the parameters!
    enum source_type : uint8_t {
        SOURCE_GPS = 0,
        SOURCE_MAVLINK_SYSTEM_TIME = 1,
        SOURCE_HW = 2,
        SOURCE_NONE = 3,
    };

    /*
      get clock in UTC microseconds.  Returns false if it is not available.
     */
    bool get_utc_usec(uint64_t &usec) const;

    // get the UTC clock and the local AP_HAL::micros64() time it was read at
    bool get_utc_and_local_usec(uint64_t &utc_usec, uint64_t &local_usec) const;

    // set the system time.  If the time has already been set by
    // something better (according to source_type), this set will be
    // ignored.  Once set by the GPS, further GPS times trim the clock
    // rate so it follows GPS time without ever stepping backwards.
    void set_utc_usec(uint64_t time_utc_usec, source_type type);

    // true when the clock is following GPS time to within a few milliseconds
    bool gps_locked() const;

    // as above, at an explicit local time; used by the unit tests
    void set_utc_usec_at(uint64_t time_utc_usec, source_type type, uint64_t local_us);
    bool get_utc_usec_at(uint64_t local_us, uint64_t &usec) const;
    bool gps_locked_at(uint64_t local_us) const;

    // return source type currently in use
    source_type get_source_type() const { return rtc_source_type; }

    /*
      get time in UTC hours, minutes, seconds and milliseconds
     */
    bool get_system_clock_utc(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const;
    
    bool get_local_time(uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const;

    uint32_t get_time_utc(int32_t hour, int32_t min, int32_t sec, int32_t ms);

    // get date and time.  Returns true on success and fills in year, month, day, hour, min, sec and ms
    // year is the regular Gregorian year, month is 0~11, day is 1~31, hour is 0~23, minute is 0~59, second is 0~60 (1 leap second), ms is 0~999
    bool get_date_and_time_utc(uint16_t& year, uint8_t& month, uint8_t& day, uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const;

    // get singleton instance
    static AP_RTC *get_singleton() {
        return _singleton;
    }

    // allow threads to lock against RTC update
    HAL_Semaphore &get_semaphore(void) {
        return rsem;
    }

    bool clock_s_to_date_fields(const uint32_t utc_sec32, uint16_t& year, uint8_t& month, uint8_t& day, uint8_t &hour, uint8_t &min, uint8_t &sec, uint8_t &wday) const;
    uint32_t date_fields_to_clock_s(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) const;

    // emit an RTC message to the onboard logs
    void Log_RTC();

#if AP_RTC_REMOTE_CLOCKS_ENABLED
    // clock offsets of other MAVLink components
    AP_RTC_RemoteClocks &remote_clocks() { return _remote_clocks; }
#endif

private:

    static AP_RTC *_singleton;

#if AP_RTC_REMOTE_CLOCKS_ENABLED
    AP_RTC_RemoteClocks _remote_clocks;
#endif
    mutable HAL_Semaphore rsem;

    source_type rtc_source_type = SOURCE_NONE;

    // UTC = anchor_utc_us + dt + dt * rate, dt = local time since anchor_local_us
    uint64_t anchor_local_us;
    uint64_t anchor_utc_us;
    float rate;                 // clock rate trim (fraction)
    float drift;                // integrated rate trim, the local crystal error (fraction)
    float err_filt_s;           // filtered GPS minus clock error
    uint8_t gps_samples;        // GPS samples since the last step, saturating
    uint64_t last_gps_local_us;
    uint64_t last_slew_side_effects_us;
#if AP_RTC_LOGGING_ENABLED
    uint64_t last_slew_log_us;
#endif

    uint64_t utc_at(uint64_t local_us) const;
    void step_to(uint64_t time_utc_usec, uint64_t local_us);
    void slew_towards(uint64_t time_utc_usec, uint64_t local_us);

    void clock_ms_to_hms_fields(const uint64_t time_ms, uint8_t &hour, uint8_t &min, uint8_t &sec, uint16_t &ms) const;

    static bool _is_leap(uint32_t y);
    static uint32_t _timegm(struct tm &tm);
};

namespace AP {
    AP_RTC &rtc();
};

#endif  // AP_RTC_ENABLED
