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
#include <AP_Math/crc.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// AP_GPS::params is protected; the tests need a Params to hand to the backend
class AP_GPS_TestAccess : public AP_GPS
{
public:
    using AP_GPS::params;
};
static AP_GPS_TestAccess test_gps;

class AP_GPS_NMEA_Test
{
public:
    int32_t parse_decimal_100(const char *p) const
    {
        return AP_GPS_NMEA::_parse_decimal_100(p);
    }

    AP_GPS_NMEA_Test()
    {
        memset(&state, 0, sizeof(state));
        driver = NEW_NOTHROW AP_GPS_NMEA(test_gps, test_gps.params[0], state, nullptr);
    }
    ~AP_GPS_NMEA_Test()
    {
        delete driver;
    }

    // feed one sentence body (without '$'/'#', '*' and checksum); the checksum
    // is computed the way _decode() accumulates it, so the terms reach the
    // handlers that publish into state
    void send(const char *body, bool unicore = false)
    {
        char line[600];
        if (unicore) {
            const uint32_t crc = crc_crc32(0, (const uint8_t *)body, strlen(body));
            hal.util->snprintf(line, sizeof(line), "#%s*%08x\r\n", body, (unsigned)crc);
        } else {
            uint8_t parity = 0;
            for (const char *c = body; *c != 0; c++) {
                parity ^= *c;
            }
            hal.util->snprintf(line, sizeof(line), "$%s*%02X\r\n", body, parity);
        }
        for (const char *c = line; *c != 0; c++) {
            driver->_decode(*c);
        }
    }

    void expect_agrica(bool v)
    {
        driver->_expect_agrica = v;
    }

    AP_GPS::GPS_State state;
    AP_GPS_NMEA *driver;
};

static const char *GGA_OK = "GPGGA,083433.00,3957.4902,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,";
static const char *RMC_OK = "GPRMC,083433.00,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,";
static const int32_t LAT_OK = 399581700;   // 39 deg 57.4902 min
static const int32_t LNG_OK = 1163129616;  // 116 deg 18.7777 min

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

TEST(AP_GPS_NMEA, parse_decimal_100_overlong_fields_saturate)
{
    AP_GPS_NMEA_Test test;

    /* strtol() saturates on these; the *100 must not wrap */
    ASSERT_EQ(INT32_MAX, test.parse_decimal_100("99999999999999999999"));
    ASSERT_EQ(INT32_MIN, test.parse_decimal_100("-99999999999999999999"));
    /* fractional digits must not push a just-in-range value over the edge */
    ASSERT_EQ(INT32_MAX, test.parse_decimal_100("21474836.9"));
    ASSERT_EQ(INT32_MIN, test.parse_decimal_100("-21474836.9"));
    /* normal values unchanged */
    ASSERT_EQ(2147483500, test.parse_decimal_100("21474835"));
}

TEST(AP_GPS_NMEA, valid_position_is_published)
{
    AP_GPS_NMEA_Test test;
    test.send(GGA_OK);
    EXPECT_EQ(LAT_OK, test.state.location.lat);
    EXPECT_EQ(LNG_OK, test.state.location.lng);
    EXPECT_EQ(AP_GPS_FixType::FIX_3D, test.state.status);
}

TEST(AP_GPS_NMEA, impossible_latitude_discards_sentence)
{
    AP_GPS_NMEA_Test test;
    /* five integer degree digits: wrapped the uint8_t accumulator and
       overflowed degrees*1e7, publishing -196.8 degrees */
    test.send("GPGGA,083433.00,99999.0000,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, test.state.location.lat);
    EXPECT_EQ(0, test.state.location.lng);
    EXPECT_EQ(AP_GPS_FixType::NO_GPS, test.state.status);  // nothing published

    /* minutes >= 60 */
    test.send("GPGGA,083433.00,3960.0000,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, test.state.location.lat);

    /* more than 180 degrees of longitude */
    test.send("GPGGA,083433.00,3957.4902,N,18118.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, test.state.location.lng);

    /* a good sentence afterwards is still accepted */
    test.send(GGA_OK);
    EXPECT_EQ(LAT_OK, test.state.location.lat);
}

TEST(AP_GPS_NMEA, impossible_time_discards_sentence)
{
    AP_GPS_NMEA_Test test;
    test.send(GGA_OK);           // 3D fix so RMC's make_gps_time() runs
    /* 9999999.00 is not a UTC time; *10 overflowed int32 and set a GPS week */
    test.send("GPRMC,9999999.00,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,");
    EXPECT_EQ(0, test.state.time_week);
    EXPECT_EQ(0U, test.state.time_week_ms);

    /* the rejected time must not linger: an RMC with an empty time field
       afterwards used to multiply the stale 9999999.00 and overflow */
    test.send("GPRMC,,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,");
    EXPECT_EQ(2179, test.state.time_week);
    EXPECT_EQ(549291000U, test.state.time_week_ms);   // the last valid time (GGA_OK), not 9999999.00

    /* month 99 in the date field */
    test.send("GPRMC,083433.00,A,3957.4902,N,11618.7777,E,0.11,133.18,169921,,");
    EXPECT_EQ(2179, test.state.time_week);

    /* a good RMC still sets the time */
    test.send(RMC_OK);
    EXPECT_EQ(2179, test.state.time_week);
}

TEST(AP_GPS_NMEA, overlong_speed_saturates_instead_of_wrapping)
{
    AP_GPS_NMEA_Test test;
    /* _parse_decimal_100() clamps to INT32_MAX; *514 then wrapped to -514
       and published 0 m/s. Saturation is the documented clamp behaviour. */
    test.send("GPVTG,133.18,T,120.79,M,99999999,N,0.20,K,A");
    EXPECT_NEAR(float(int32_t((int64_t(INT32_MAX) * 514) / 1000)) * 0.01f,
                test.state.ground_speed, 1.0f);
}

AP_GTEST_MAIN()
