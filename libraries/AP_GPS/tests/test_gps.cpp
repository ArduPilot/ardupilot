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
#include <memory>
#include <string>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class AP_GPS_NMEA_Test
{
public:
    int32_t parse_decimal_100(const char *p) const
    {
        return AP_GPS_NMEA::_parse_decimal_100(p);
    }

    // drivers are always heap allocated, which zeroes them; a stack
    // instance would leave members like _last_KSXT_pos_ms uninitialised
    static AP_GPS_NMEA *make(AP_GPS::GPS_State &state);

    // feed a sentence body, adding '$', the checksum and line ending
    static void feed(AP_GPS_NMEA &nmea, const std::string &body, bool with_checksum=true)
    {
        uint8_t cs = 0;
        for (char c : body) {
            cs ^= uint8_t(c);
        }
        char tail[8];
        snprintf(tail, sizeof(tail), "*%02X\r\n", cs);
        const std::string s = "$" + body + (with_checksum ? tail : "\r\n");
        for (char c : s) {
            nmea._decode(c);
        }
    }
    static const double *ksxt_fields(const AP_GPS_NMEA &nmea) { return nmea._ksxt.fields; }
    static bool is_rmc(const AP_GPS_NMEA &nmea) { return nmea._sentence_type == AP_GPS_NMEA::_GPS_SENTENCE_RMC; }
    // the RMC term number that lands on a given GGA term in the dispatch switch
    static uint16_t rmc_term_aliasing_gga(uint8_t gga_term)
    {
        return AP_GPS_NMEA::_GPS_SENTENCE_GGA - AP_GPS_NMEA::_GPS_SENTENCE_RMC + gga_term;
    }
    static std::string phd_bytes(const AP_GPS_NMEA &nmea) { return std::string((const char *)&nmea._phd, sizeof(nmea._phd)); }
    static std::string ksxt_bytes(const AP_GPS_NMEA &nmea) { return std::string((const char *)&nmea._ksxt, sizeof(nmea._ksxt)); }
#if AP_GPS_NMEA_UNICORE_ENABLED
    static std::string agrica_bytes(const AP_GPS_NMEA &nmea) { return std::string((const char *)&nmea._agrica, sizeof(nmea._agrica)); }
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

    /* Negative numbers in (-1, 0) range, with possible round/truncate */
    ASSERT_EQ(0, test.parse_decimal_100("-0.0"));
    ASSERT_EQ(0, test.parse_decimal_100("-0.00"));
    ASSERT_EQ(0, test.parse_decimal_100("-0.001"));
    ASSERT_EQ(-1, test.parse_decimal_100("-0.006"));
    ASSERT_EQ(-50, test.parse_decimal_100("-0.50"));

    /* Negative numbers with possible round/truncate */
    ASSERT_EQ(-100, test.parse_decimal_100("-1.0"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.00"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1.001"));
    ASSERT_EQ(-101, test.parse_decimal_100("-1.006"));

    /* Integer numbers */
    ASSERT_EQ(100, test.parse_decimal_100("1"));
    ASSERT_EQ(-100, test.parse_decimal_100("-1"));
}

static AP_GPS gps;

#if GPS_MAX_RECEIVERS > 1

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

// NMEA parser tests don't need more than one receiver
static AP_GPS::Params nmea_params;

AP_GPS_NMEA *AP_GPS_NMEA_Test::make(AP_GPS::GPS_State &state)
{
    AP_GPS_NMEA *nmea = NEW_NOTHROW AP_GPS_NMEA(gps, nmea_params, state, nullptr);
    EXPECT_NE(nullptr, nmea);
    return nmea;
}

static const char *KSXT_21_FIELDS = "KSXT,20211016083433.00,116.31296102,39.95817066,49.4911,223.57,-11.32,330.19,0.024,,1,3,28,27,,,,-0.012,0.021,0.020,,";

TEST(AP_GPS_NMEA, ksxt_fields)
{
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;

    AP_GPS_NMEA_Test::feed(nmea, KSXT_21_FIELDS);
    const double *f = AP_GPS_NMEA_Test::ksxt_fields(nmea);
    EXPECT_NEAR(116.31296102, f[1], 1e-4);
    EXPECT_NEAR(39.95817066, f[2], 1e-4);
    EXPECT_NEAR(0.020, f[18], 1e-4);

#if AP_GPS_NMEA_UNICORE_ENABLED
    // a 22nd field must not be written past the end of _ksxt.fields
    const std::string agrica = AP_GPS_NMEA_Test::agrica_bytes(nmea);
    AP_GPS_NMEA_Test::feed(nmea, std::string(KSXT_21_FIELDS) + ",12345.678");
    EXPECT_EQ(agrica, AP_GPS_NMEA_Test::agrica_bytes(nmea));
#endif
}

// a sentence with many terms must not alias into the fields of another
// sentence type (issue #35181)
TEST(AP_GPS_NMEA, long_sentences_do_not_alias)
{
    const char *heads[] = { "GPRMC", "GPGGA", "GPVTG", "GPHDT", "PHD", "GPTHS", "KSXT" };
    for (const char *head : heads) {
        for (uint16_t terms = 1; terms <= 300; terms++) {
            AP_GPS::GPS_State state {};
            std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
            AP_GPS_NMEA &nmea = *nmea_ptr;
            const std::string phd = AP_GPS_NMEA_Test::phd_bytes(nmea);
            const std::string ksxt = AP_GPS_NMEA_Test::ksxt_bytes(nmea);
#if AP_GPS_NMEA_UNICORE_ENABLED
            const std::string agrica = AP_GPS_NMEA_Test::agrica_bytes(nmea);
#endif
            std::string body = head;
            for (uint16_t i = 0; i < terms; i++) {
                body += ",7";
            }
            // fields are stored before the checksum is checked
            AP_GPS_NMEA_Test::feed(nmea, body, false);
            if (strcmp(head, "PHD") != 0) {
                EXPECT_EQ(phd, AP_GPS_NMEA_Test::phd_bytes(nmea)) << head << " terms=" << terms;
            }
            if (strcmp(head, "KSXT") != 0) {
                EXPECT_EQ(ksxt, AP_GPS_NMEA_Test::ksxt_bytes(nmea)) << head << " terms=" << terms;
            }
#if AP_GPS_NMEA_UNICORE_ENABLED
            EXPECT_EQ(agrica, AP_GPS_NMEA_Test::agrica_bytes(nmea)) << head << " terms=" << terms;
#endif
        }
    }
}

TEST(AP_GPS_NMEA, long_rmc_keeps_position)
{
    const std::string rmc = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";

    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;
    AP_GPS_NMEA_Test::feed(nmea, rmc);
    const int32_t lat = state.location.lat;
    const int32_t lng = state.location.lng;
    EXPECT_NE(0, lat);

    // pad with empty terms up to the RMC terms that alias onto GGA
    // latitude, N/S and longitude (GGA terms 2..4)
    std::string body = rmc;
    for (uint16_t i = 12; i < AP_GPS_NMEA_Test::rmc_term_aliasing_gga(2); i++) {
        body += ",";
    }
    body += ",1234.000,S,05000.000";
    AP_GPS_NMEA_Test::feed(nmea, body);
    EXPECT_EQ(lat, state.location.lat);
    EXPECT_EQ(lng, state.location.lng);
}

TEST(AP_GPS_NMEA, term_number_does_not_wrap)
{
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;

    // 65536 terms would wrap a uint16_t term number back to 0 and
    // re-detect the sentence type from the next term
    std::string body = "GPRMC";
    body.append(65536, ',');
    body += KSXT_21_FIELDS;
    AP_GPS_NMEA_Test::feed(nmea, body, false);
    EXPECT_TRUE(AP_GPS_NMEA_Test::is_rmc(nmea));
}

static const char *GGA_OK = "GPGGA,083433.00,3957.4902,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,";
static const char *RMC_OK = "GPRMC,083433.00,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,";
static const int32_t LAT_OK = 399581700;   // 39 deg 57.4902 min
static const int32_t LNG_OK = 1163129616;  // 116 deg 18.7777 min

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
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;
    AP_GPS_NMEA_Test::feed(nmea, GGA_OK);
    EXPECT_EQ(LAT_OK, state.location.lat);
    EXPECT_EQ(LNG_OK, state.location.lng);
    EXPECT_EQ(AP_GPS_FixType::FIX_3D, state.status);
}

TEST(AP_GPS_NMEA, impossible_latitude_discards_sentence)
{
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;
    /* five integer degree digits: wrapped the uint8_t accumulator and
       overflowed degrees*1e7, publishing -196.8 degrees */
    AP_GPS_NMEA_Test::feed(nmea, "GPGGA,083433.00,99999.0000,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, state.location.lat);
    EXPECT_EQ(0, state.location.lng);
    EXPECT_EQ(AP_GPS_FixType::NO_GPS, state.status);  // nothing published

    /* minutes >= 60 */
    AP_GPS_NMEA_Test::feed(nmea, "GPGGA,083433.00,3960.0000,N,11618.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, state.location.lat);

    /* more than 180 degrees of longitude */
    AP_GPS_NMEA_Test::feed(nmea, "GPGGA,083433.00,3957.4902,N,18118.7777,E,1,10,1.2,0049.49,M,0.0,M,,");
    EXPECT_EQ(0, state.location.lng);

    /* a good sentence afterwards is still accepted */
    AP_GPS_NMEA_Test::feed(nmea, GGA_OK);
    EXPECT_EQ(LAT_OK, state.location.lat);
}

TEST(AP_GPS_NMEA, impossible_time_discards_sentence)
{
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;
    AP_GPS_NMEA_Test::feed(nmea, GGA_OK);           // 3D fix so RMC's make_gps_time() runs
    /* 9999999.00 is not a UTC time; *10 overflowed int32 and set a GPS week */
    AP_GPS_NMEA_Test::feed(nmea, "GPRMC,9999999.00,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,");
    EXPECT_EQ(0, state.time_week);
    EXPECT_EQ(0U, state.time_week_ms);

    /* the rejected time must not linger: an RMC with an empty time field
       afterwards used to multiply the stale 9999999.00 and overflow */
    AP_GPS_NMEA_Test::feed(nmea, "GPRMC,,A,3957.4902,N,11618.7777,E,0.11,133.18,161021,,");
    EXPECT_EQ(2179, state.time_week);
    EXPECT_EQ(549291000U, state.time_week_ms);   // the last valid time (GGA_OK), not 9999999.00

    /* month 99 in the date field */
    AP_GPS_NMEA_Test::feed(nmea, "GPRMC,083433.00,A,3957.4902,N,11618.7777,E,0.11,133.18,169921,,");
    EXPECT_EQ(2179, state.time_week);

    /* a good RMC still sets the time */
    AP_GPS_NMEA_Test::feed(nmea, RMC_OK);
    EXPECT_EQ(2179, state.time_week);
}

TEST(AP_GPS_NMEA, overlong_speed_saturates_instead_of_wrapping)
{
    AP_GPS::GPS_State state {};
    std::unique_ptr<AP_GPS_NMEA> nmea_ptr {AP_GPS_NMEA_Test::make(state)};
    AP_GPS_NMEA &nmea = *nmea_ptr;
    /* _parse_decimal_100() clamps to INT32_MAX; *514 then wrapped to -514
       and published 0 m/s. Saturation is the documented clamp behaviour. */
    AP_GPS_NMEA_Test::feed(nmea, "GPVTG,133.18,T,120.79,M,99999999,N,0.20,K,A");
    EXPECT_NEAR(float(int32_t((int64_t(INT32_MAX) * 514) / 1000)) * 0.01f,
                state.ground_speed, 1.0f);
}

AP_GTEST_MAIN()
