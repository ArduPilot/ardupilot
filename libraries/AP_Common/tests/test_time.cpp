#include <AP_gtest.h>
#include <AP_Common/time.h>

static struct tm make_tm(int year, int mon, int mday, int hour, int min, int sec)
{
    struct tm t{};
    t.tm_year = year - 1900;
    t.tm_mon  = mon - 1;   // 0-based
    t.tm_mday = mday;
    t.tm_hour = hour;
    t.tm_min  = min;
    t.tm_sec  = sec;
    return t;
}

// Unix epoch: 1970-01-01 00:00:00
TEST(ap_mktime, Epoch)
{
    struct tm t = make_tm(1970, 1, 1, 0, 0, 0);
    EXPECT_EQ((time_t)0, ap_mktime(&t));
}

// tm_year < 70 must return (time_t)-1
TEST(ap_mktime, PreEpoch)
{
    struct tm t = make_tm(1969, 12, 31, 23, 59, 59);
    EXPECT_EQ((time_t)-1, ap_mktime(&t));

    t = make_tm(1900, 1, 1, 0, 0, 0);
    EXPECT_EQ((time_t)-1, ap_mktime(&t));
}

// tm_year == 70 is the boundary that must succeed
TEST(ap_mktime, PreEpochBoundary)
{
    struct tm t = make_tm(1970, 1, 1, 0, 0, 0);
    EXPECT_NE((time_t)-1, ap_mktime(&t));
}

// Well-known epoch: 2000-01-01 00:00:00 UTC = 946684800
TEST(ap_mktime, Y2K)
{
    struct tm t = make_tm(2000, 1, 1, 0, 0, 0);
    EXPECT_EQ((time_t)946684800, ap_mktime(&t));
}

// Seconds, minutes and hours accumulate correctly
TEST(ap_mktime, TimeOfDay)
{
    struct tm t = make_tm(1970, 1, 1, 1, 2, 3);
    EXPECT_EQ((time_t)(1*3600 + 2*60 + 3), ap_mktime(&t));
}

// Day-of-month offset (1-based: day 1 adds 0 extra days)
TEST(ap_mktime, DayOfMonth)
{
    struct tm jan1  = make_tm(1970, 1,  1, 0, 0, 0);
    struct tm jan15 = make_tm(1970, 1, 15, 0, 0, 0);
    EXPECT_EQ((time_t)(14 * 86400), ap_mktime(&jan15) - ap_mktime(&jan1));
}

// All 12 months accumulate the right number of days in a non-leap year (2001)
TEST(ap_mktime, AllMonthsNonLeap)
{
    // days elapsed from Jan 1 to the first of each month in a common year
    const int days[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    struct tm jan1 = make_tm(2001, 1, 1, 0, 0, 0);
    for (int mon = 1; mon <= 12; mon++) {
        struct tm t = make_tm(2001, mon, 1, 0, 0, 0);
        EXPECT_EQ((time_t)(days[mon - 1] * 86400), ap_mktime(&t) - ap_mktime(&jan1))
            << "month " << mon;
    }
}

// Leap year (div by 4, not 100): 2004-02-29 exists and March 1 is day 61
TEST(ap_mktime, LeapYearRegular)
{
    struct tm jan1  = make_tm(2004, 1,  1, 0, 0, 0);
    struct tm feb29 = make_tm(2004, 2, 29, 0, 0, 0);
    struct tm mar1  = make_tm(2004, 3,  1, 0, 0, 0);

    EXPECT_EQ((time_t)(59 * 86400), ap_mktime(&feb29) - ap_mktime(&jan1));
    EXPECT_EQ((time_t)(60 * 86400), ap_mktime(&mar1)  - ap_mktime(&jan1));
}

// Leap year (div by 400): 2000-02-29 exists
TEST(ap_mktime, LeapYearDiv400)
{
    struct tm jan1  = make_tm(2000, 1,  1, 0, 0, 0);
    struct tm feb29 = make_tm(2000, 2, 29, 0, 0, 0);
    struct tm mar1  = make_tm(2000, 3,  1, 0, 0, 0);

    EXPECT_EQ((time_t)(59 * 86400), ap_mktime(&feb29) - ap_mktime(&jan1));
    EXPECT_EQ((time_t)(60 * 86400), ap_mktime(&mar1)  - ap_mktime(&jan1));
}

// Century non-leap year (div by 100 but not 400): 2100 has no Feb 29
TEST(ap_mktime, CenturyNonLeap)
{
    struct tm jan1 = make_tm(2100, 1, 1, 0, 0, 0);
    struct tm mar1 = make_tm(2100, 3, 1, 0, 0, 0);
    // Feb has only 28 days → March 1 is day 60 (59 days after Jan 1)
    EXPECT_EQ((time_t)(59 * 86400), ap_mktime(&mar1) - ap_mktime(&jan1));
}

// The month loop resets m=0 and y++ when m exceeds 11.
// tm_mon=12 means "13th month" = January of the following year internally.
TEST(ap_mktime, MonthLoopRollover)
{
    // 1971-01-01: 365 days after epoch
    struct tm t = make_tm(1970, 1, 1, 0, 0, 0);
    t.tm_mon = 12; // 13 iterations: Dec wraps m→0, y→1971, then Jan added
    EXPECT_EQ((time_t)(365 * 86400), ap_mktime(&t));
}

AP_GTEST_MAIN()
