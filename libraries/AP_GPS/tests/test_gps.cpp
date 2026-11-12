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
#include <AP_GPS/AP_GPS_Blended.h>

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

#if GPS_MAX_RECEIVERS > 1
static AP_GPS gps;

class AP_GPS_Test
{
public:
    static void setup()
    {
#if AP_GPS_BLENDED_ENABLED
        const uint8_t blended = GPS_BLENDED_INSTANCE;
        gps.drivers[blended] = new AP_GPS_Blended(gps, gps.params[blended], gps.state[blended], gps.timing[blended]);
#endif
        for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
            gps.params[i].type.set(AP_GPS::GPS_TYPE_SBF);
            gps.state[i].num_sats = 20;
            gps.state[i].last_gps_time_ms = AP_HAL::millis();
        }
    }

    static void teardown()
    {
#if AP_GPS_BLENDED_ENABLED
        delete gps.drivers[GPS_BLENDED_INSTANCE];
        gps.drivers[GPS_BLENDED_INSTANCE] = nullptr;
#endif
    }

    static uint8_t select(AP_GPS_FixType first, AP_GPS_FixType second, uint8_t primary, uint8_t auto_switch = 1)
    {
        gps.state[0].status = first;
        gps.state[1].status = second;
        gps.primary_instance = primary;
        gps._auto_switch.set(auto_switch);
        gps._primary.set(0);
        gps.update_primary();
        return gps.primary_instance;
    }

#if AP_GPS_BLENDED_ENABLED
    static AP_GPS_FixType blend(AP_GPS_FixType first, AP_GPS_FixType second)
    {
        gps._blend_mask.set(1); // Horizontal position accuracy.
        for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
            gps.state[i].last_gps_time_ms = 1;
            gps.state[i].have_horizontal_accuracy = true;
            gps.state[i].horizontal_accuracy = 0.25f;
            gps.state[i].location.lat = 100000000;
            gps.state[i].location.lng = 200000000;
        }
        EXPECT_EQ(select(first, second, 0, 2), GPS_BLENDED_INSTANCE);
        return gps.state[GPS_BLENDED_INSTANCE].status;
    }
#endif
};

TEST(AP_GPS, primary_fix_priority)
{
    AP_GPS_Test::setup();
    const AP_GPS_FixType ordered[] = {
        AP_GPS_FixType::NO_GPS, AP_GPS_FixType::NONE, AP_GPS_FixType::FIX_2D,
        AP_GPS_FixType::FIX_3D, AP_GPS_FixType::DGPS, AP_GPS_FixType::PPP,
        AP_GPS_FixType::RTK_FLOAT, AP_GPS_FixType::RTK_FIXED, AP_GPS_FixType::STATIC,
    };
    // Check every pair in both receiver positions and from either primary.
    for (uint8_t i = 0; i < ARRAY_SIZE(ordered); i++) {
        for (uint8_t j = 0; j < ARRAY_SIZE(ordered); j++) {
            for (uint8_t primary = 0; primary < 2; primary++) {
                EXPECT_EQ(AP_GPS_Test::select(ordered[i], ordered[j], primary),
                          i == j ? primary : (i > j ? 0 : 1));
            }
#if AP_GPS_BLENDED_ENABLED
            if (i >= 3 && j >= 3) {
                // Also check recovery from a blended solution.
                EXPECT_EQ(AP_GPS_Test::select(ordered[i], ordered[j], GPS_BLENDED_INSTANCE), i >= j ? 0 : 1);
            }
#endif
        }
    }
    // Explicit primary selection must still override quality ranking.
    EXPECT_EQ(AP_GPS_Test::select(AP_GPS_FixType::PPP, AP_GPS_FixType::RTK_FIXED, 1, 0), 0);
    EXPECT_EQ(AP_GPS_Test::select(AP_GPS_FixType::PPP, AP_GPS_FixType::RTK_FIXED, 1, 4), 0);
    AP_GPS_Test::teardown();
}

#if AP_GPS_BLENDED_ENABLED
TEST(AP_GPS, blended_fix_priority)
{
    AP_GPS_Test::setup();
    const AP_GPS_FixType statuses[] = {
        AP_GPS_FixType::RTK_FIXED, AP_GPS_FixType::RTK_FLOAT,
        AP_GPS_FixType::PPP, AP_GPS_FixType::DGPS, AP_GPS_FixType::FIX_3D,
    };
    for (const auto status : statuses) {
        const auto expected = (status == AP_GPS_FixType::RTK_FIXED || status == AP_GPS_FixType::RTK_FLOAT) ?
                              status : AP_GPS_FixType::PPP;
        EXPECT_EQ(AP_GPS_Test::blend(AP_GPS_FixType::PPP, status), expected);
        EXPECT_EQ(AP_GPS_Test::blend(status, AP_GPS_FixType::PPP), expected);
    }
    AP_GPS_Test::teardown();
}
#endif
#endif

AP_GTEST_MAIN()
