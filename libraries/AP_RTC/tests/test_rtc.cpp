/*
  Unit tests for AP_RTC.

  Tests within the AP_RTC suite run in definition order, and the RTC
  never allows time to move backwards, so each test installs times
  later than any installed by earlier tests; the set_clock() helper
  guarantees this by using a fresh, later day for every call.
 */

#include <AP_gtest.h>

#include <AP_Common/time.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_RTC rtc;
#if HAL_LOGGING_ENABLED
// set_utc_usec() logs RTC updates via AP::logger():
static AP_Logger logger;
#endif
// set_utc_usec() updates the MAVLink signing timestamp via gcs():
static GCS_Dummy _gcs;

// slack allowed between an expected value and one derived from the
// running clock; the clock advances in real time between installing a
// time and querying it.  Test cases must not place an expected value
// within this distance of a discontinuity (second rollover, rollover
// to a following day, ...).
static const uint32_t clock_slack_ms = 100;

// return a fresh day-number (days since epoch), later than any
// returned before
static uint32_t next_test_day()
{
    static uint32_t days = 20000;  // ~2024; the RTC rejects times before 2022
    days++;
    return days;
}

// utc microseconds for a time-of-day on a given day-number
static uint64_t day_time_to_utc_usec(uint32_t days, uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms)
{
    const uint64_t tod_ms = ((hour*60UL + min)*60UL + sec)*1000ULL + ms;
    return (days*86400000ULL + tod_ms)*1000ULL;
}

// install a time-of-day into the RTC on a fresh day, returning the
// installed time
static uint64_t set_clock(uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms)
{
    const uint64_t utc_usec = day_time_to_utc_usec(next_test_day(), hour, min, sec, ms);
    AP::rtc().set_utc_usec(utc_usec, AP_RTC::SOURCE_GPS);
    return utc_usec;
}

// must run before any test installs a time: with no time source every
// time accessor must fail
TEST(AP_RTC, unset_clock)
{
    uint64_t utc_usec;
    EXPECT_FALSE(AP::rtc().get_utc_usec(utc_usec));

    uint8_t hour, min, sec;
    uint16_t ms;
    EXPECT_FALSE(AP::rtc().get_system_clock_utc(hour, min, sec, ms));
    EXPECT_FALSE(AP::rtc().get_local_time(hour, min, sec, ms));

    uint16_t year;
    uint8_t month, day;
    EXPECT_FALSE(AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms));

    EXPECT_EQ(0U, AP::rtc().get_time_utc(13, 0, 0, 0));

    EXPECT_EQ(AP_RTC::SOURCE_NONE, AP::rtc().get_source_type());

    // a source not in the allowed-types mask (GPS-only by default)
    // must be rejected:
    const uint64_t t_2024 = day_time_to_utc_usec(19800, 12, 0, 0, 0);
    AP::rtc().set_utc_usec(t_2024, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);
    EXPECT_FALSE(AP::rtc().get_utc_usec(utc_usec));
    EXPECT_EQ(AP_RTC::SOURCE_NONE, AP::rtc().get_source_type());

    // times before the oldest acceptable date (2022-01-01) must be
    // rejected:
    const uint64_t t_2020 = day_time_to_utc_usec(18300, 12, 0, 0, 0);
    AP::rtc().set_utc_usec(t_2020, AP_RTC::SOURCE_GPS);
    EXPECT_FALSE(AP::rtc().get_utc_usec(utc_usec));
    EXPECT_EQ(AP_RTC::SOURCE_NONE, AP::rtc().get_source_type());
}

// clock_s_to_date_fields() and date_fields_to_clock_s() are pure
// conversions, so exact values can be tested
TEST(AP_RTC, date_field_conversions)
{
    static const struct {
        uint32_t utc_sec;
        uint16_t year;
        uint8_t month;  // 0-11
        uint8_t day;    // 1-31
        uint8_t hour;
        uint8_t min;
        uint8_t sec;
        uint8_t wday;   // 0-6, Sunday=0
    } cases[] {
        { 0U,          1970, 0,  1,  0,  0,  0,  4 },  // epoch (a Thursday)
        { 951825600U,  2000, 1, 29, 12,  0,  0,  2 },  // div-400 leap year
        { 1709251199U, 2024, 1, 29, 23, 59, 59,  4 },  // leap day, last second
        { 1783514096U, 2026, 6,  8, 12, 34, 56,  3 },  // an ordinary Wednesday
        { 2147483648U, 2038, 0, 19,  3, 14,  8,  2 },  // beyond the int32 epoch limit
        { 4102444800U, 2100, 0,  1,  0,  0,  0,  5 },  // div-100 non-leap century
        { 4294967295U, 2106, 1,  7,  6, 28, 15,  0 },  // uint32 epoch limit
    };

    for (const auto &tc : cases) {
        SCOPED_TRACE(tc.utc_sec);

        uint16_t year;
        uint8_t month, day, hour, min, sec, wday;
        ASSERT_TRUE(AP::rtc().clock_s_to_date_fields(tc.utc_sec, year, month, day, hour, min, sec, wday));
        EXPECT_EQ(tc.year, year);
        EXPECT_EQ(int(tc.month), int(month));
        EXPECT_EQ(int(tc.day), int(day));
        EXPECT_EQ(int(tc.hour), int(hour));
        EXPECT_EQ(int(tc.min), int(min));
        EXPECT_EQ(int(tc.sec), int(sec));
        EXPECT_EQ(int(tc.wday), int(wday));

        EXPECT_EQ(tc.utc_sec, AP::rtc().date_fields_to_clock_s(tc.year, tc.month, tc.day, tc.hour, tc.min, tc.sec));
    }

    // an invalid month must not convert:
    EXPECT_EQ(0U, AP::rtc().date_fields_to_clock_s(2024, 13, 1, 0, 0, 0));
}

// date_fields_to_clock_s() (and so _timegm()/_is_leap()) must agree
// with ap_mktime() across the whole range representable in the uint32
// seconds-since-epoch it returns.  ap_mktime() is an independent
// in-tree implementation - it counts months in a loop and tests for
// leap years inline - so it catches errors in the closed-form day
// count and in the _is_leap() bit trick alike.
TEST(AP_RTC, date_fields_to_clock_s_sweep)
{
    // uint32 seconds since the epoch runs out during 2106-02-07:
    for (uint16_t year = 1970; year <= 2105; year++) {
        for (uint8_t month = 0; month < 12; month++) {
            // 1st, 10th, 19th, 28th: every month has these, and the
            // 28th/1st pair straddles the February leap day
            for (uint8_t day = 1; day <= 28; day += 9) {
                SCOPED_TRACE(year*10000 + month*100 + day);

                struct tm tm {};
                tm.tm_year = year - 1900;
                tm.tm_mon = month;
                tm.tm_mday = day;
                tm.tm_hour = 7;
                tm.tm_min = 11;
                tm.tm_sec = 23;
                const time_t expected = ap_mktime(&tm);
                ASSERT_NE(time_t(-1), expected);

                EXPECT_EQ(uint32_t(expected),
                          AP::rtc().date_fields_to_clock_s(year, month, day, 7, 11, 23));
            }
        }
    }
}

// February's length exercises _is_leap() through the _timegm() day
// table; the div-100 and div-400 cases are the ones a naive
// "divisible by 4" test gets wrong
TEST(AP_RTC, february_length)
{
    static const struct {
        uint16_t year;
        uint8_t days;
    } cases[] {
        { 1972, 29 },  // first leap year after the epoch
        { 2023, 28 },  // not divisible by 4
        { 2024, 29 },  // divisible by 4
        { 2000, 29 },  // divisible by 400: a leap year
        { 2100, 28 },  // divisible by 100 but not 400: not a leap year
        { 2104, 29 },  // divisible by 4, latest in uint32 range
    };

    for (const auto &tc : cases) {
        SCOPED_TRACE(tc.year);

        const uint32_t feb_1 = AP::rtc().date_fields_to_clock_s(tc.year, 1, 1, 0, 0, 0);
        const uint32_t mar_1 = AP::rtc().date_fields_to_clock_s(tc.year, 2, 1, 0, 0, 0);
        EXPECT_EQ(uint32_t(tc.days), (mar_1 - feb_1) / 86400U);
    }
}

// dates before the epoch cannot be represented in the returned uint32
// and must be rejected rather than wrapping around
TEST(AP_RTC, pre_epoch_rejected)
{
    EXPECT_EQ(0U, AP::rtc().date_fields_to_clock_s(1969, 11, 31, 23, 59, 59));
    EXPECT_EQ(0U, AP::rtc().date_fields_to_clock_s(1900, 0, 1, 0, 0, 0));
    EXPECT_EQ(0U, AP::rtc().date_fields_to_clock_s(1, 0, 1, 0, 0, 0));

    // the epoch itself is representable, and is the boundary case:
    EXPECT_EQ(0U, AP::rtc().date_fields_to_clock_s(1970, 0, 1, 0, 0, 0));
    EXPECT_EQ(1U, AP::rtc().date_fields_to_clock_s(1970, 0, 1, 0, 0, 1));
}

// get_time_utc() returns the number of milliseconds until a target
// time-of-day.  Each of hour/min/sec/ms may be -1, meaning "ignore
// this field".
TEST(AP_RTC, get_time_utc)
{
    static const struct {
        const char *desc;
        // time-of-day to install as "now":
        uint8_t curr_hour;
        uint8_t curr_min;
        uint8_t curr_sec;
        uint16_t curr_ms;
        // arguments to get_time_utc(), -1 meaning ignore:
        int32_t hour;
        int32_t min;
        int32_t sec;
        int32_t ms;
        uint32_t expected_ms;
    } cases[] {
        // fully-specified targets:
        { "fully-specified future time",
          12, 0, 0, 250,    13, 15, 10, 0,    4509750 },
        { "fully-specified time just past rolls over a full day",
          13, 0, 10, 250,   13, 0, 5, 0,      86394750 },
        { "fully-specified time crossing midnight",
          23, 30, 0, 250,   0, 10, 0, 0,      2399750 },

        // leading ignored fields:
        { "seconds-only target later this minute",
          12, 0, 10, 250,   -1, -1, 40, 0,    29750 },
        { "seconds-only target just past rolls over to next minute",
          12, 0, 40, 250,   -1, -1, 10, 0,    29750 },
        { "minutes-only target just past rolls over to next hour",
          12, 30, 0, 250,   -1, 10, 0, 0,     2399750 },
        { "ms-only target later this second",
          12, 0, 10, 250,   -1, -1, -1, 750,  500 },
        { "ms-only target just past rolls over to next second",
          12, 0, 10, 250,   -1, -1, -1, 100,  850 },
        { "ignored hour with min:sec just past rolls over one hour",
          12, 30, 45, 250,  -1, 30, 40, 0,    3594750 },
        { "ignored ms with target next second",
          12, 0, 10, 250,   -1, -1, 11, -1,   750 },

#if 0   // these cases document the intended handling of ignored
        // values below the largest specified element and fail before
        // the get_time_utc() ignored-values fix
        // (https://github.com/ArduPilot/ardupilot/pull/33667); remove
        // this #if 0 when that PR lands
        { "ignored minutes just before target hour must not delay a day",
          12, 59, 30, 250,  13, -1, 0, 0,     29750 },
        { "ignored minutes within target hour waits for next minute",
          13, 30, 15, 250,  13, -1, 0, 0,     44750 },
        { "ignored minutes with target hour in the past waits until tomorrow",
          14, 30, 0, 250,   13, -1, 0, 0,     80999750 },
        { "ignored minutes in target hour's final minute must wait until tomorrow",
          13, 59, 30, 250,  13, -1, 0, 0,     82829750 },
        { "carry from ignored seconds must not escape the specified minute",
          23, 30, 59, 500,  -1, 30, -1, 0,    3540500 },
        { "carry through ignored minutes must not escape the specified hour",
          13, 59, 59, 250,  13, -1, -1, 0,    82800750 },
        { "ignored minutes and seconds with future target hour",
          12, 30, 45, 250,  14, -1, -1, -1,   5354750 },
        { "ignored seconds within target minute leaves only a ms delta",
          12, 30, 15, 250,  -1, 30, -1, 900,  650 },
        { "ignored seconds with target minute in the past",
          12, 30, 45, 250,  -1, 10, -1, 0,    2354750 },
        { "ignored ms with current second as target returns zero",
          12, 0, 10, 250,   -1, -1, 10, -1,   0 },
        { "ignored ms and seconds with future target minute",
          12, 30, 45, 250,  -1, 45, -1, -1,   854750 },
        { "ignored seconds with target hour in the past rolls over a day",
          14, 30, 0, 250,   13, 0, -1, 0,     80999750 },
#endif

        // everything ignored:
        { "all fields ignored returns zero",
          12, 0, 10, 250,   -1, -1, -1, -1,   0 },
    };

    // the first accepted set_utc_usec() triggers one-time expensive
    // work (lazily initialising storage to save the MAVLink signing
    // timestamp, updating the hardware RTC); do a throwaway install
    // so that cost is not measured against clock_slack_ms below
    set_clock(0, 0, 0, 0);

    for (const auto &tc : cases) {
        SCOPED_TRACE(tc.desc);

        set_clock(tc.curr_hour, tc.curr_min, tc.curr_sec, tc.curr_ms);

        // ensure the time was actually installed:
        uint8_t hour, min, sec;
        uint16_t ms;
        ASSERT_TRUE(AP::rtc().get_system_clock_utc(hour, min, sec, ms));
        ASSERT_EQ(int(tc.curr_hour), int(hour));
        ASSERT_EQ(int(tc.curr_min), int(min));
        ASSERT_EQ(int(tc.curr_sec), int(sec));

        const uint32_t delay_ms = AP::rtc().get_time_utc(tc.hour, tc.min, tc.sec, tc.ms);
        EXPECT_NEAR(tc.expected_ms, delay_ms, clock_slack_ms);
    }
}

TEST(AP_RTC, get_utc_usec)
{
    const uint64_t installed = set_clock(12, 34, 56, 250);

    uint64_t first, second;
    ASSERT_TRUE(AP::rtc().get_utc_usec(first));
    ASSERT_TRUE(AP::rtc().get_utc_usec(second));

    // the clock must be close to the installed time, and running
    // forwards:
    EXPECT_GE(first, installed);
    EXPECT_NEAR(0, double(first - installed)/1000, clock_slack_ms);
    EXPECT_GE(second, first);

    uint8_t hour, min, sec;
    uint16_t ms;
    ASSERT_TRUE(AP::rtc().get_system_clock_utc(hour, min, sec, ms));
    EXPECT_EQ(12, hour);
    EXPECT_EQ(34, min);
    EXPECT_EQ(56, sec);
    EXPECT_NEAR(250, ms, clock_slack_ms);
}

TEST(AP_RTC, get_local_time)
{
    static const struct {
        const char *desc;
        uint8_t curr_hour;
        uint8_t curr_min;
        int16_t tz_min;
        uint8_t local_hour;
        uint8_t local_min;
    } cases[] {
        { "UTC",                     12, 0,     0,   12, 0 },
        { "positive whole hours",    12, 0,   600,   22, 0 },
        { "negative part hour",      12, 0,   -90,   10, 30 },
        { "offset crossing midnight", 23, 30,  120,   1, 30 },
    };

    for (const auto &tc : cases) {
        SCOPED_TRACE(tc.desc);

        set_clock(tc.curr_hour, tc.curr_min, 0, 250);
        rtc.tz_min.set(tc.tz_min);

        uint8_t hour, min, sec;
        uint16_t ms;
        ASSERT_TRUE(AP::rtc().get_local_time(hour, min, sec, ms));
        EXPECT_EQ(int(tc.local_hour), int(hour));
        EXPECT_EQ(int(tc.local_min), int(min));
        EXPECT_EQ(0, sec);
    }

    rtc.tz_min.set(0);
}

TEST(AP_RTC, set_utc_usec_priority)
{
    // allow all source types so the priority rules are tested rather
    // than the allowed-types mask:
    rtc.allowed_types.set((1U << AP_RTC::SOURCE_GPS) |
                          (1U << AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME) |
                          (1U << AP_RTC::SOURCE_HW));

    const uint64_t t1 = set_clock(12, 0, 0, 0);
    ASSERT_EQ(AP_RTC::SOURCE_GPS, AP::rtc().get_source_type());

    uint64_t now;

    // while disarmed, a worse source than the current one must be
    // rejected:
    const uint64_t t2 = day_time_to_utc_usec(next_test_day(), 12, 0, 0, 0);
    AP::rtc().set_utc_usec(t2, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);
    EXPECT_EQ(AP_RTC::SOURCE_GPS, AP::rtc().get_source_type());
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_LT(now, t2);

    // while armed, moving time backwards must be rejected, even from
    // the same source:
    hal.util->set_soft_armed(true);
    AP::rtc().set_utc_usec(t1 - 3600*1000000ULL, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_GE(now, t1);
    hal.util->set_soft_armed(false);

    // while disarmed, the GPS steps a large error out in either direction:
    AP::rtc().set_utc_usec(t1 - 3600*1000000ULL, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_LT(now, t1);
    EXPECT_NEAR(0, double(int64_t(now - (t1 - 3600*1000000ULL)))/1000, clock_slack_ms);

    // while armed, a same-source time is only slewed towards, never
    // stepped to...
    const uint64_t t3 = day_time_to_utc_usec(next_test_day(), 12, 0, 0, 0);
    hal.util->set_soft_armed(true);
    AP::rtc().set_utc_usec(t3, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_LT(now, t3);

    // ... and stepped to once disarmed again:
    hal.util->set_soft_armed(false);
    AP::rtc().set_utc_usec(t3, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_GE(now, t3);
    EXPECT_NEAR(0, double(now - t3)/1000, clock_slack_ms);

    rtc.allowed_types.set(1U << AP_RTC::SOURCE_GPS);
}

TEST(AP_RTC, get_date_and_time_utc)
{
    const uint64_t installed = set_clock(12, 34, 56, 250);

    // derive the expected calendar date using clock_s_to_date_fields,
    // which date_field_conversions verified against known dates:
    uint16_t exp_year;
    uint8_t exp_month, exp_day, exp_hour, exp_min, exp_sec, exp_wday;
    ASSERT_TRUE(AP::rtc().clock_s_to_date_fields(installed / 1000000ULL, exp_year, exp_month, exp_day, exp_hour, exp_min, exp_sec, exp_wday));

    uint16_t year;
    uint8_t month, day, hour, min, sec;
    uint16_t ms;
    ASSERT_TRUE(AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms));
    EXPECT_EQ(exp_year, year);
    EXPECT_EQ(int(exp_month), int(month));
    EXPECT_EQ(int(exp_day), int(day));
    EXPECT_EQ(12, hour);
    EXPECT_EQ(34, min);
    EXPECT_EQ(56, sec);
    EXPECT_NEAR(250, ms, clock_slack_ms);
}

/*
  The tests below drive the clock at synthetic local times, later than
  any real time used above, so they must stay at the end of the suite.
 */

// a fresh synthetic local time base, later than any returned before.
// Each base is also within one day of the last, so start_sim() always steps forward
static uint64_t next_local_base_us()
{
    static uint64_t base_us = AP_HAL::micros64() + 1000000000ULL;
    base_us += 2000000000ULL;  // less than the one day next_test_day() advances
    return base_us;
}

// deterministic +-1ms GPS time noise
static int64_t gps_noise_us(uint32_t i)
{
    return int64_t((i * 7919U) % 2001U) - 1000;
}

struct ClockSim {
    uint64_t local0_us;
    uint64_t utc0_us;
    double crystal_error;   // local clock rate error, e.g. 50e-6 runs fast
    uint64_t local_at(double t_s) const { return local0_us + uint64_t(t_s * (1.0 + crystal_error) * 1.0e6); }
    uint64_t utc_at(double t_s) const { return utc0_us + uint64_t(t_s * 1.0e6); }
};

// step the clock to a fresh day at a fresh local time base
static ClockSim start_sim(double crystal_error)
{
    ClockSim sim {
        next_local_base_us(),
        day_time_to_utc_usec(next_test_day(), 12, 0, 0, 0),
        crystal_error,
    };
    AP::rtc().set_utc_usec_at(sim.utc0_us, AP_RTC::SOURCE_GPS, sim.local0_us);
    return sim;
}

// feed 5Hz noisy GPS times from t0_s to t1_s, checking the clock never
// runs backwards; returns the clock error at t1_s in seconds
static double feed_gps(const ClockSim &sim, double t0_s, double t1_s, int64_t gps_offset_us=0)
{
    uint64_t last_utc = 0;
    uint32_t i = 0;
    for (double t = t0_s; t <= t1_s; t += 0.2, i++) {
        const uint64_t local_us = sim.local_at(t);
        uint64_t before;
        EXPECT_TRUE(AP::rtc().get_utc_usec_at(local_us, before));
        AP::rtc().set_utc_usec_at(sim.utc_at(t) + gps_offset_us + gps_noise_us(i), AP_RTC::SOURCE_GPS, local_us);
        uint64_t after, mid;
        EXPECT_TRUE(AP::rtc().get_utc_usec_at(local_us, after));
        EXPECT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(t + 0.1), mid));
        // never backwards, and no jump at an update:
        EXPECT_GE(before, last_utc);
        EXPECT_EQ(before, after);
        EXPECT_GT(mid, after);
        last_utc = mid;
    }
    uint64_t now;
    EXPECT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(t1_s), now));
    return (int64_t(now) - int64_t(sim.utc_at(t1_s))) * 1.0e-6;
}

// a fast or slow crystal is slewed out without stepping, armed or disarmed
TEST(AP_RTC, slew_follows_drifting_gps)
{
    for (const double crystal_error : { 50.0e-6, -50.0e-6, 20.0e-6, -20.0e-6 }) {
        for (const bool armed : { false, true }) {
            SCOPED_TRACE(crystal_error * 1e6);
            SCOPED_TRACE(armed);
            const ClockSim sim = start_sim(crystal_error);
            hal.util->set_soft_armed(armed);
            const double err_s = feed_gps(sim, 0.2, 900.0);
            EXPECT_NEAR(0.0, err_s, 0.001);
            EXPECT_TRUE(AP::rtc().gps_locked_at(sim.local_at(900.0)));
            hal.util->set_soft_armed(false);
        }
    }
}

// a small forward error while disarmed is slewed, not stepped
TEST(AP_RTC, small_forward_error_slews)
{
    const ClockSim sim = start_sim(0.0);
    feed_gps(sim, 0.2, 300.0);
    uint64_t before, after;
    const uint64_t local_us = sim.local_at(300.2);
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(local_us, before));
    AP::rtc().set_utc_usec_at(sim.utc_at(300.2) + 20000, AP_RTC::SOURCE_GPS, local_us);
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(local_us, after));
    EXPECT_EQ(before, after);
    // and is removed by slewing:
    EXPECT_NEAR(0.0, feed_gps(sim, 300.4, 900.0, 20000) - 0.02, 0.002);
}

// a large forward error steps while disarmed, but only slews while armed
TEST(AP_RTC, large_forward_error_steps_only_disarmed)
{
    const ClockSim sim = start_sim(0.0);
    feed_gps(sim, 0.2, 60.0);

    hal.util->set_soft_armed(true);
    uint64_t now;
    AP::rtc().set_utc_usec_at(sim.utc_at(60.2) + 1000000, AP_RTC::SOURCE_GPS, sim.local_at(60.2));
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(60.2), now));
    EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(60.2))) * 1.0e-6, 0.005);
    EXPECT_FALSE(AP::rtc().gps_locked_at(sim.local_at(60.2)));

    hal.util->set_soft_armed(false);
    AP::rtc().set_utc_usec_at(sim.utc_at(60.4) + 1000000, AP_RTC::SOURCE_GPS, sim.local_at(60.4));
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(60.4), now));
    EXPECT_EQ(sim.utc_at(60.4) + 1000000, now);
}

// while armed, a GPS time behind the clock is slewed to at the limited rate, never stepped back
TEST(AP_RTC, never_steps_back_armed)
{
    const ClockSim sim = start_sim(0.0);
    feed_gps(sim, 0.2, 60.0);
    hal.util->set_soft_armed(true);

    uint64_t start, end;
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(60.0), start));
    // GPS time now one second behind: feed_gps checks the clock never runs backwards
    feed_gps(sim, 60.2, 160.0, -1000000);
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(160.0), end));
    // the clock slowed by at most the rate limit
    EXPECT_GE(double(end - start), 100.0e6 * (1.0 - 500.0e-6) - 1);
    EXPECT_LT(double(end - start), 100.0e6);
    EXPECT_FALSE(AP::rtc().gps_locked_at(sim.local_at(160.0)));
    hal.util->set_soft_armed(false);
}

// while disarmed, a GPS time well behind the clock is stepped back to
TEST(AP_RTC, disarmed_steps_back_to_gps)
{
    const ClockSim sim = start_sim(0.0);
    feed_gps(sim, 0.2, 60.0);
    const uint64_t gps_utc = sim.utc_at(60.2) - 1000000;
    AP::rtc().set_utc_usec_at(gps_utc, AP_RTC::SOURCE_GPS, sim.local_at(60.2));
    uint64_t now;
    ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(60.2), now));
    EXPECT_EQ(gps_utc, now);
}

// a clock set ahead by a worse source is stepped back to GPS time while disarmed
TEST(AP_RTC, gps_replaces_time_ahead_while_disarmed)
{
    // a separate RTC, as the shared one has already been set by the GPS.  It is
    // allocated with NEW_NOTHROW so it starts zeroed, like the static one
    AP_RTC *fresh = NEW_NOTHROW AP_RTC;
    ASSERT_NE(nullptr, fresh);
    fresh->allowed_types.set((1U << AP_RTC::SOURCE_GPS) | (1U << AP_RTC::SOURCE_HW));
    const uint64_t local_us = next_local_base_us();
    const uint64_t gps_utc = day_time_to_utc_usec(next_test_day(), 12, 0, 0, 0);
    fresh->set_utc_usec_at(gps_utc + 10000000, AP_RTC::SOURCE_HW, local_us);
    ASSERT_EQ(AP_RTC::SOURCE_HW, fresh->get_source_type());
    fresh->set_utc_usec_at(gps_utc, AP_RTC::SOURCE_GPS, local_us);
    EXPECT_EQ(AP_RTC::SOURCE_GPS, fresh->get_source_type());
    uint64_t now;
    ASSERT_TRUE(fresh->get_utc_usec_at(local_us, now));
    EXPECT_EQ(gps_utc, now);
    delete fresh;
}

// without GPS the lock is held for two minutes, however long the clock had followed GPS
TEST(AP_RTC, lock_held_two_minutes)
{
    for (const double crystal_error : { 50.0e-6, -50.0e-6 }) {
        for (const double train_s : { 3.0, 20.0, 100.0, 900.0 }) {
            SCOPED_TRACE(crystal_error * 1e6);
            SCOPED_TRACE(train_s);
            const ClockSim sim = start_sim(crystal_error);
            feed_gps(sim, 0.2, train_s);
            ASSERT_TRUE(AP::rtc().gps_locked_at(sim.local_at(train_s)));
            const double last_locked_s = train_s + 119.5;
            EXPECT_TRUE(AP::rtc().gps_locked_at(sim.local_at(last_locked_s)));
            EXPECT_FALSE(AP::rtc().gps_locked_at(sim.local_at(train_s + 120.5)));
            uint64_t now;
            ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(last_locked_s), now));
            EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(last_locked_s))) * 1.0e-3, 10.0);
        }
    }
}

// an expired lock does not return with the first GPS sample after an outage
TEST(AP_RTC, expired_lock_gained_again)
{
    for (const double train_s : { 3.0, 20.0, 100.0 }) {
        for (const double outage_s : { 300.0, 600.0, 900.0 }) {
            SCOPED_TRACE(train_s);
            SCOPED_TRACE(outage_s);
            const ClockSim sim = start_sim(50.0e-6);
            feed_gps(sim, 0.2, train_s);
            ASSERT_TRUE(AP::rtc().gps_locked_at(sim.local_at(train_s)));

            const double return_s = train_s + outage_s;
            feed_gps(sim, return_s, return_s);
            EXPECT_FALSE(AP::rtc().gps_locked_at(sim.local_at(return_s)));

            // it is gained once the error is slewed out, and can then be held
            const double lost_s = return_s + 300.0;
            feed_gps(sim, return_s + 0.2, lost_s);
            ASSERT_TRUE(AP::rtc().gps_locked_at(sim.local_at(lost_s)));
            uint64_t now;
            ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(lost_s + 119.5), now));
            EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(lost_s + 119.5))) * 1.0e-3, 10.0);
        }
    }
}

// slewing out an offset must not change the rate the clock runs at without GPS
TEST(AP_RTC, slew_leaves_no_rate_error)
{
    for (const int64_t offset_us : { 80000, -80000 }) {
        SCOPED_TRACE(offset_us);
        const ClockSim sim = start_sim(0.0);
        feed_gps(sim, 0.2, 600.0);
        hal.util->set_soft_armed(true);
        feed_gps(sim, 600.2, 800.0, offset_us);
        hal.util->set_soft_armed(false);
        ASSERT_TRUE(AP::rtc().gps_locked_at(sim.local_at(800.0)));
        uint64_t now;
        ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(919.5), now));
        EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(919.5) + offset_us)) * 1.0e-3, 5.0);
    }
}

// GPS lost straight after an offset leaves no lock that can drift beyond the tolerance
TEST(AP_RTC, gps_lost_after_offset)
{
    for (const double crystal_error : { 50.0e-6, -50.0e-6 }) {
        for (const int64_t offset_us : { 11000, 16000, 25000, 45000 }) {
            for (const uint8_t num_samples : { 1, 3, 10 }) {
                SCOPED_TRACE(crystal_error * 1e6);
                SCOPED_TRACE(offset_us);
                SCOPED_TRACE(int(num_samples));
                const ClockSim sim = start_sim(crystal_error);
                feed_gps(sim, 0.2, 60.0);
                const double lost_s = 60.2 + (num_samples - 1) * 0.2;
                feed_gps(sim, 60.2, lost_s + 0.01, offset_us);
                const double end_s = lost_s + 119.5;
                if (!AP::rtc().gps_locked_at(sim.local_at(end_s))) {
                    continue;
                }
                uint64_t now;
                ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(end_s), now));
                EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(end_s) + offset_us)) * 1.0e-3, AP_RTC_REMOTE_CLOCK_TOLERANCE_MS);
            }
        }
    }
}

// short bursts of GPS can't carry a lock along while the drift between them accumulates
TEST(AP_RTC, intermittent_gps_within_tolerance)
{
    for (const double crystal_error : { 50.0e-6, -50.0e-6 }) {
        for (const uint8_t num_samples : { 1, 5, 15 }) {
            SCOPED_TRACE(crystal_error * 1e6);
            SCOPED_TRACE(int(num_samples));
            const ClockSim sim = start_sim(crystal_error);
            feed_gps(sim, 0.2, 60.0);
            hal.util->set_soft_armed(true);     // so the drift can only be slewed out
            double t = 60.0;
            for (uint8_t i=0; i<10; i++) {
                t += 119.0;
                const double lost_s = t + (num_samples - 1) * 0.2;
                feed_gps(sim, t, lost_s + 0.01);
                t = lost_s;
                const double end_s = t + 119.0;
                if (!AP::rtc().gps_locked_at(sim.local_at(end_s))) {
                    continue;
                }
                uint64_t now;
                ASSERT_TRUE(AP::rtc().get_utc_usec_at(sim.local_at(end_s), now));
                EXPECT_NEAR(0.0, (int64_t(now) - int64_t(sim.utc_at(end_s))) * 1.0e-3, AP_RTC_REMOTE_CLOCK_TOLERANCE_MS);
            }
            hal.util->set_soft_armed(false);
        }
    }
}

// the lock is gained below 2 ms of error but only lost above 12 ms
TEST(AP_RTC, lock_hysteresis)
{
    for (const int64_t jump_us : { 10000, 25000 }) {
        SCOPED_TRACE(jump_us);
        const ClockSim sim = start_sim(0.0);
        feed_gps(sim, 0.2, 300.0);
        ASSERT_TRUE(AP::rtc().gps_locked_at(sim.local_at(300.0)));
        feed_gps(sim, 300.2, 310.0, jump_us);
        EXPECT_EQ(jump_us < 12000, AP::rtc().gps_locked_at(sim.local_at(310.0)));
    }
}

AP_GTEST_MAIN()
