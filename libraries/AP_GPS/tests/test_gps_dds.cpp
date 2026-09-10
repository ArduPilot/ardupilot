/*
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

/*
  Exercises AP_GPS_DDS in isolation by calling its handler directly with
  synthetic samples. This is NOT the DDS wiring: no subscriber exists yet and
  nothing in AP_DDS calls handle_navsatfix(). It only proves the backend's own
  mechanics -- field population and the read()/new-data handshake.
 */

#include <AP_gtest.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_GPS/AP_GPS_DDS.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_GPS_DDS_ENABLED

// AP_GPS panics if constructed twice, so share one instance across tests
static AP_GPS &shared_gps()
{
    static AP_GPS gps;
    return gps;
}

static AP_GPS_DDS::NavSatFix make_sample()
{
    AP_GPS_DDS::NavSatFix pkt {};
    pkt.gps_week = 2350;
    pkt.gps_tow_ms = 123456789;
    pkt.fix_type = AP_GPS_FixType::FIX_3D;
    pkt.num_sats = 14;
    pkt.latitude_deg = 47.3977419;
    pkt.longitude_deg = 8.5455938;
    pkt.altitude_amsl_m = 12.34f;
    pkt.have_velocity = true;
    pkt.velocity_ned = Vector3f{1.5f, -2.5f, 0.75f};
    pkt.have_vertical_velocity = true;
    pkt.have_accuracy = true;
    pkt.horizontal_accuracy = 0.8f;
    pkt.vertical_accuracy = 1.6f;
    pkt.speed_accuracy = 0.25f;
    pkt.have_dop = true;
    pkt.hdop = 1.55f;
    pkt.vdop = 2.10f;
    return pkt;
}

TEST(AP_GPS_DDS, read_reports_new_data_once)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    // nothing pushed yet
    EXPECT_FALSE(backend.read());

    backend.handle_navsatfix(make_sample());

    // one push yields exactly one true, then false until the next push
    EXPECT_TRUE(backend.read());
    EXPECT_FALSE(backend.read());
    EXPECT_FALSE(backend.read());

    backend.handle_navsatfix(make_sample());
    EXPECT_TRUE(backend.read());
    EXPECT_FALSE(backend.read());
}

TEST(AP_GPS_DDS, populates_state_fields)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    backend.handle_navsatfix(make_sample());

    EXPECT_EQ(AP_GPS::GPS_OK_FIX_3D, state.status);
    EXPECT_EQ(2350, state.time_week);
    EXPECT_EQ(123456789u, state.time_week_ms);
    EXPECT_EQ(14, state.num_sats);

    EXPECT_EQ(473977419, state.location.lat);
    EXPECT_EQ(85455938, state.location.lng);
    EXPECT_EQ(1234, state.location.alt);

    EXPECT_EQ(155, state.hdop);
    EXPECT_EQ(210, state.vdop);

    EXPECT_FLOAT_EQ(1.5f, state.velocity.x);
    EXPECT_FLOAT_EQ(-2.5f, state.velocity.y);
    EXPECT_FLOAT_EQ(0.75f, state.velocity.z);
    EXPECT_TRUE(state.have_vertical_velocity);

    // derived from the horizontal velocity components
    EXPECT_NEAR(2.9154759f, state.ground_speed, 1.0e-4);
    EXPECT_NEAR(300.9637565f, state.ground_course, 1.0e-3);

    EXPECT_TRUE(state.have_horizontal_accuracy);
    EXPECT_TRUE(state.have_vertical_accuracy);
    EXPECT_TRUE(state.have_speed_accuracy);
    EXPECT_FLOAT_EQ(0.8f, state.horizontal_accuracy);
    EXPECT_FLOAT_EQ(1.6f, state.vertical_accuracy);
    EXPECT_FLOAT_EQ(0.25f, state.speed_accuracy);
}

TEST(AP_GPS_DDS, no_fix_maps_to_no_fix_status)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    AP_GPS_DDS::NavSatFix pkt = make_sample();
    pkt.fix_type = AP_GPS_FixType::NO_GPS;
    backend.handle_navsatfix(pkt);

    EXPECT_TRUE(backend.read());
    EXPECT_EQ(AP_GPS::NO_FIX, state.status);
}

TEST(AP_GPS_DDS, rejects_out_of_range_and_nan_positions)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    AP_GPS_DDS::NavSatFix pkt = make_sample();
    pkt.latitude_deg = nan("");
    backend.handle_navsatfix(pkt);
    EXPECT_FALSE(backend.read());

    pkt = make_sample();
    pkt.longitude_deg = 181.0;
    backend.handle_navsatfix(pkt);
    EXPECT_FALSE(backend.read());

    pkt = make_sample();
    pkt.latitude_deg = -91.0;
    backend.handle_navsatfix(pkt);
    EXPECT_FALSE(backend.read());

    // a rejected sample must not have touched state
    EXPECT_EQ(0, state.location.lat);
    EXPECT_EQ(0, state.num_sats);
}

TEST(AP_GPS_DDS, optional_fields_left_alone_when_absent)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    AP_GPS_DDS::NavSatFix pkt = make_sample();
    pkt.have_velocity = false;
    pkt.have_accuracy = false;
    pkt.have_dop = false;
    backend.handle_navsatfix(pkt);

    EXPECT_TRUE(backend.read());
    EXPECT_EQ(473977419, state.location.lat);

    EXPECT_FLOAT_EQ(0.0f, state.velocity.x);
    EXPECT_EQ(0, state.hdop);
    EXPECT_EQ(0, state.vdop);
    EXPECT_FALSE(state.have_horizontal_accuracy);
    EXPECT_FALSE(state.have_vertical_accuracy);
    EXPECT_FALSE(state.have_speed_accuracy);
}

TEST(AP_GPS_DDS, reports_its_name)
{
    AP_GPS::Params params;
    AP_GPS::GPS_State state {};
    state.instance = 0;
    AP_GPS_DDS backend(shared_gps(), params, state, nullptr);

    EXPECT_STREQ("DDS", backend.name());
}

#endif  // AP_GPS_DDS_ENABLED

AP_GTEST_MAIN()
