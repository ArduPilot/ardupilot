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
#include <AP_GPS/AP_GPS_UBLOX.h>

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
}


#if AP_GPS_UBLOX_ENABLED
class AP_GPS_UBLOX_Test
{
public:
    /*
      build a NAV-PVT for a UTC time as sent by a receiver which uses
      leap_sec leap seconds, and return the week derived from its date
      or -1 if no week could be derived
     */
    static int32_t week_from_utc(time_t utc, uint8_t leap_sec, uint16_t itow_frac_ms=0,
                                 uint8_t valid=UBX_PVT_VALID_DATE|UBX_PVT_VALID_TIME)
    {
        struct tm tm {};
        gmtime_r(&utc, &tm);
        AP_GPS_UBLOX::ubx_nav_pvt pvt {};
        pvt.year = tm.tm_year + 1900;
        pvt.month = tm.tm_mon + 1;
        pvt.day = tm.tm_mday;
        pvt.hour = tm.tm_hour;
        pvt.min = tm.tm_min;
        pvt.sec = tm.tm_sec;
        pvt.valid = valid;
        const uint64_t gps_sec = uint64_t(utc) + leap_sec - gps_epoch_unix;
        pvt.itow = (gps_sec % sec_per_week) * 1000U + itow_frac_ms;
        uint16_t week;
        if (!AP_GPS_UBLOX::_week_from_pvt_date(pvt, week)) {
            return -1;
        }
        return week;
    }

    static int32_t true_week(time_t utc, uint8_t leap_sec)
    {
        return (uint64_t(utc) + leap_sec - gps_epoch_unix) / sec_per_week;
    }

    static const uint32_t gps_epoch_unix = 315964800;  // 1980-01-06 00:00:00 UTC
    static const uint32_t sec_per_week = 604800;
};

TEST(AP_GPS_UBLOX, week_from_pvt_date)
{
    AP_GPS_UBLOX_Test test;

    const time_t times[] {
        1577836800,  // 2020-01-01 00:00:00
        1709210096,  // 2024-02-29 12:34:56
        1792411200,  // 2026-10-19 12:00:00
        1924991999,  // 2030-12-31 23:59:59
        4102444799,  // 2099-12-31 23:59:59
    };
    const uint16_t rollover_weeks[] { 2100, 2391, 2392, 2500, 3000 };

    // receivers may use a leap second count other than the one we assume
    for (uint8_t leap_sec = 16; leap_sec <= 20; leap_sec++) {
        for (const uint16_t frac_ms : { 0, 999 }) {
            for (const time_t utc : times) {
                EXPECT_EQ(test.true_week(utc, leap_sec), test.week_from_utc(utc, leap_sec, frac_ms));
            }
            // either side of the week rollover
            for (const uint16_t week : rollover_weeks) {
                const time_t rollover = time_t(test.gps_epoch_unix) + time_t(week) * test.sec_per_week - leap_sec;
                for (int8_t ofs = -5; ofs <= 5; ofs++) {
                    const time_t utc = rollover + ofs;
                    EXPECT_EQ(test.true_week(utc, leap_sec), test.week_from_utc(utc, leap_sec, frac_ms));
                }
            }
        }
    }

    // date and time must both be flagged valid
    EXPECT_EQ(-1, test.week_from_utc(1792411200, 18, 0, 0));
    EXPECT_EQ(-1, test.week_from_utc(1792411200, 18, 0, UBX_PVT_VALID_DATE));
    EXPECT_EQ(-1, test.week_from_utc(1792411200, 18, 0, UBX_PVT_VALID_TIME));

    // implausible dates are rejected
    EXPECT_EQ(-1, test.week_from_utc(1546300800, 18));  // 2019-01-01
    EXPECT_EQ(-1, test.week_from_utc(4102444800, 18));  // 2100-01-01
}
#endif  // AP_GPS_UBLOX_ENABLED

AP_GTEST_MAIN()
