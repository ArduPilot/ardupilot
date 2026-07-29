/*
  Unit tests for AP_RTC.

  Tests within the AP_RTC suite run in definition order, and the RTC
  never allows time to move backwards, so each test installs times
  later than any installed by earlier tests; the set_clock() helper
  guarantees this by using a fresh, later day for every call.
 */

#include <AP_gtest.h>

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
        { "ignored minutes just before target hour must not delay a day",
          12, 59, 30, 250,  13, -1, 0, 0,     29750 },
        { "ignored minutes within target hour waits for next minute",
          13, 30, 15, 250,  13, -1, 0, 0,     44750 },
        { "ignored minutes with target hour in the past waits until tomorrow",
          14, 30, 0, 250,   13, -1, 0, 0,     80999750 },
        { "ignored minutes in target hour's final minute rolls into next hour",
          13, 59, 30, 250,  13, -1, 0, 0,     29750 },
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

    // moving time backwards must be rejected, even from the same
    // source:
    AP::rtc().set_utc_usec(t1 - 3600*1000000ULL, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_GE(now, t1);

    // while armed, even same-source updates must be rejected...
    const uint64_t t3 = day_time_to_utc_usec(next_test_day(), 12, 0, 0, 0);
    hal.util->set_soft_armed(true);
    AP::rtc().set_utc_usec(t3, AP_RTC::SOURCE_GPS);
    ASSERT_TRUE(AP::rtc().get_utc_usec(now));
    EXPECT_LT(now, t3);

    // ... and accepted once disarmed again:
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

AP_GTEST_MAIN()
