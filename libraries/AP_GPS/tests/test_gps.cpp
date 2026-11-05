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

#if AP_GPS_NMEA_TESEO_ENABLED
    static bool decode(AP_GPS_NMEA &nmea, char c) { return nmea._decode(c); }
    static void set_teseo(AP_GPS_NMEA &nmea) { nmea._is_teseo = true; }
    static uint8_t kfcov_have(const AP_GPS_NMEA &nmea) { return nmea._pstmkfcov.have; }
    static uint8_t pvraw_have(const AP_GPS_NMEA &nmea) { return nmea._pstmpvraw.have; }
    static uint32_t reply_param(const AP_GPS_NMEA &nmea) { return nmea._teseo_reply.param; }
    static uint32_t reply_value(const AP_GPS_NMEA &nmea) { return nmea._teseo_reply.value; }
    static float reply_value_float(const AP_GPS_NMEA &nmea) { return nmea._teseo_reply.value_float; }
#endif
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

#if AP_GPS_NMEA_TESEO_ENABLED
static AP_GPS gps;
static AP_GPS::Params gps_params;

class AP_GPS_NMEA_Teseo_Test : public ::testing::Test
{
protected:
    AP_GPS::GPS_State state {};
    AP_GPS_NMEA nmea {gps, gps_params, state, nullptr};

    void SetUp() override
    {
        // the driver marks a used sentence with a time of 1, which is
        // only stale once millis() is past the 150ms freshness window
        while (AP_HAL::millis() < 200) {
        }
    }

    // feed a sentence, without '$' or checksum, to the parser. True
    // if it completed a fix
    bool feed(const char *body)
    {
        uint8_t cs = 0;
        for (const char *p = body; *p; p++) {
            cs ^= uint8_t(*p);
        }
        char tail[8];
        hal.util->snprintf(tail, sizeof(tail), "*%02X\r\n", unsigned(cs));
        bool fix = AP_GPS_NMEA_Test::decode(nmea, '$');
        for (const char *p = body; *p; p++) {
            fix |= AP_GPS_NMEA_Test::decode(nmea, *p);
        }
        for (const char *p = tail; *p; p++) {
            fix |= AP_GPS_NMEA_Test::decode(nmea, *p);
        }
        return fix;
    }

    // a sentence with num_terms terms of "1" after its name
    void feed_long(const char *name, uint16_t num_terms)
    {
        char body[1024];
        size_t len = strlen(name);
        memcpy(body, name, len);
        for (uint16_t i = 0; i < num_terms && len < sizeof(body)-2; i++) {
            body[len++] = ',';
            body[len++] = '1';
        }
        body[len] = 0;
        feed(body);
    }
};

// an over-long GGA must not reach the Teseo parser, whatever the GPS type
TEST_F(AP_GPS_NMEA_Teseo_Test, long_gga)
{
    feed_long("GPGGA", 300);
    EXPECT_EQ(0U, AP_GPS_NMEA_Test::kfcov_have(nmea));
    EXPECT_EQ(0U, AP_GPS_NMEA_Test::pvraw_have(nmea));
}

// extra terms on a Teseo sentence must not be taken as another's
TEST_F(AP_GPS_NMEA_Teseo_Test, long_kfcov)
{
    AP_GPS_NMEA_Test::set_teseo(nmea);
    feed_long("PSTMKFCOV", 50);
    EXPECT_EQ(0xFFU, AP_GPS_NMEA_Test::kfcov_have(nmea));
    EXPECT_EQ(0U, AP_GPS_NMEA_Test::pvraw_have(nmea));
    EXPECT_EQ(0U, AP_GPS_NMEA_Test::reply_param(nmea));
}

TEST_F(AP_GPS_NMEA_Teseo_Test, pvraw)
{
    AP_GPS_NMEA_Test::set_teseo(nmea);
    feed("PSTMPVRAW,144056.000,5131.12414,N,00005.31484,W,2,09,1.2,043.31,M,47.0,M,-0.6,0.1,0.6");
    EXPECT_FLOAT_EQ(-0.6, state.velocity.x);
    EXPECT_FLOAT_EQ(0.1, state.velocity.y);
    // Vel_V is up, velocity is NED
    EXPECT_FLOAT_EQ(-0.6, state.velocity.z);
    EXPECT_TRUE(state.have_vertical_velocity);
}

// $PSTMSETPAR is the reply to $PSTMGETPAR, integers in hex
TEST_F(AP_GPS_NMEA_Teseo_Test, getpar_reply)
{
    AP_GPS_NMEA_Test::set_teseo(nmea);
    feed("PSTMSETPAR,3200,410000");
    EXPECT_EQ(3200U, AP_GPS_NMEA_Test::reply_param(nmea));
    EXPECT_EQ(0x410000U, AP_GPS_NMEA_Test::reply_value(nmea));
    feed("PSTMSETPAR,3201,0x8000052");
    EXPECT_EQ(3201U, AP_GPS_NMEA_Test::reply_param(nmea));
    EXPECT_EQ(0x8000052U, AP_GPS_NMEA_Test::reply_value(nmea));
    feed("PSTMSETPAR,3303,0.1");
    EXPECT_EQ(3303U, AP_GPS_NMEA_Test::reply_param(nmea));
    EXPECT_FLOAT_EQ(0.1, AP_GPS_NMEA_Test::reply_value_float(nmea));
}
#endif // AP_GPS_NMEA_TESEO_ENABLED

AP_GTEST_MAIN()
