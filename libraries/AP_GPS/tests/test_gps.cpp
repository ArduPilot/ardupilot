/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <AP_GPS/AP_GPS_NMEA.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class AP_GPS_NMEA_Test
{
public:
    int32_t parse_decimal_100(const char *p) const
    {
        return AP_GPS_NMEA::_parse_decimal_100(p);
    }
};

TEST(AP_GPS_NMEA, parse_decimal_100)
{
    AP_GPS_NMEA_Test test;

    /* Positive numbers with possible round/truncate */
    ASSERT_EQ(100, test.parse_decimal_100("1.0"));
    ASSERT_EQ(100, test.parse_decimal_100("1.00"));
    ASSERT_EQ(100, test.parse_decimal_100("1.001"));
    ASSERT_EQ(101, test.parse_decimal_100("1.006"));

    /* Positive numbers with possible round/truncate with + signal */
    ASSERT_EQ(100, test.parse_decimal_100("+1.0"));
    ASSERT_EQ(100, test.parse_decimal_100("+1.00"));
    ASSERT_EQ(100, test.parse_decimal_100("+1.001"));
    ASSERT_EQ(101, test.parse_decimal_100("+1.006"));

    /* Positive numbers in (0, 1) range, with possible round/truncate */
    ASSERT_EQ(0, test.parse_decimal_100("0.0"));
    ASSERT_EQ(0, test.parse_decimal_100("0.00"));
    ASSERT_EQ(0, test.parse_decimal_100("0.001"));
    ASSERT_EQ(1, test.parse_decimal_100("0.006"));

    /* Negative numbers with possible round/truncate */
    ASSERT_EQ(-100, test.parse_decimal_100("-1.0"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.00"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.001"));
    ASSERT_EQ(-101, test.parse_decimal_100("-1.006"));

    /* Integer numbers */
    ASSERT_EQ(100, test.parse_decimal_100("1"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1"));

    /* Zero */
    ASSERT_EQ(0, test.parse_decimal_100("0"));
    ASSERT_EQ(0, test.parse_decimal_100("0.0"));

    /* Single fractional digit */
    ASSERT_EQ(50, test.parse_decimal_100("0.5"));
    /* sign is derived from the leading '-' character, not from strtol's
     * return value, so "-0.x" inputs correctly negate the fractional part. */
    ASSERT_EQ(-50,  test.parse_decimal_100("-0.5"));
    ASSERT_EQ(-150, test.parse_decimal_100("-1.5"));

    /* Two fractional digits */
    ASSERT_EQ(55, test.parse_decimal_100("0.55"));
    ASSERT_EQ(-55, test.parse_decimal_100("-0.55"));

    /* Rounding of third digit: >= 5 rounds up, < 5 truncates */
    ASSERT_EQ(56, test.parse_decimal_100("0.555"));
    ASSERT_EQ(55, test.parse_decimal_100("0.554"));
    ASSERT_EQ(56, test.parse_decimal_100("0.556"));
    ASSERT_EQ(-56, test.parse_decimal_100("-0.555"));
    ASSERT_EQ(-55, test.parse_decimal_100("-0.554"));

    /* Leading decimal point (no integer part): ".5" treated as 0.5 */
    ASSERT_EQ(50, test.parse_decimal_100(".5"));
    ASSERT_EQ(150, test.parse_decimal_100("1.5"));

    /* Large integers */
    ASSERT_EQ(99900, test.parse_decimal_100("999"));
    ASSERT_EQ(-99900, test.parse_decimal_100("-999"));

    /* Extra digits beyond third decimal are ignored */
    ASSERT_EQ(100, test.parse_decimal_100("1.00000"));

    /* Rounding boundary: 1.004 → 100, 1.005 → 101 */
    ASSERT_EQ(100, test.parse_decimal_100("1.004"));
    ASSERT_EQ(101, test.parse_decimal_100("1.005"));
    ASSERT_EQ(-101, test.parse_decimal_100("-1.005"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.004"));
}

/* Verify parse_decimal_100 clamps overflowing inputs to INT32_MAX / INT32_MIN.
 * INT32_MAX = 2147483647; strtol("21474837") * 100 = 2147483700 > INT32_MAX.
 * The function must return INT32_MAX, not a wrapped/UB value. */
TEST(AP_GPS_NMEA, parse_decimal_100_overflow)
{
    AP_GPS_NMEA_Test test;

    /* Positive overflow: clamps to INT32_MAX */
    EXPECT_EQ(INT32_MAX, test.parse_decimal_100("21474837"));

    /* Negative overflow: clamps to INT32_MIN */
    EXPECT_EQ(INT32_MIN, test.parse_decimal_100("-21474837"));

    /* Just below overflow threshold: passes through unchanged */
    /* strtol("21474836") * 100 = 2147483600, which fits in int32_t */
    EXPECT_EQ(2147483600, test.parse_decimal_100("21474836"));

    /* Fractional digits can push past INT32_MAX/MIN even when the integer
     * part alone passes the guard: 21474836*100=2147483600 < INT32_MAX,
     * but adding 0.9*100=90 gives 2147483690 > INT32_MAX. */
    EXPECT_EQ(INT32_MAX, test.parse_decimal_100("21474836.9"));
    EXPECT_EQ(INT32_MIN, test.parse_decimal_100("-21474836.9"));

    /* Non-overflowing fractional: 21474836.4 → 2147483600 + 40 = 2147483640 */
    EXPECT_EQ(2147483640, test.parse_decimal_100("21474836.4"));
}

AP_GTEST_MAIN()
