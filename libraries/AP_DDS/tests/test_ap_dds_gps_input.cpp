#include <AP_gtest.h>

#include <AP_DDS/AP_DDS_config.h>

#if AP_DDS_GPS_INPUT_SUB_ENABLED

#include <AP_DDS/AP_DDS_GPS_Input.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// A fix as ublox_gps publishes it: NAV-PVT already scaled to SI units
static obs_msgs_msg_UbloxPvt make_msg()
{
    obs_msgs_msg_UbloxPvt msg {};
    // 2026-09-14 (a Monday) 12:00:00 UTC = GPS week 2436,
    // TOW 86400 + 43200 + 18 leap seconds
    msg.year = 2026;
    msg.month = 9;
    msg.day = 14;
    msg.hour = 12;
    msg.gpstime = 129618.2;
    msg.fixtype = 3;
    msg.numsv = 14;
    msg.lat = 52.5125623;
    msg.lon = 13.3269871;
    msg.height = 80.25;
    msg.hmsl = 36.5;
    msg.hacc = 0.8;
    msg.vacc = 1.2;
    msg.sacc = 0.15;
    msg.veln = 1.5;
    msg.vele = -2.0;
    msg.veld = 0.25;
    msg.pdop = 1.35;
    return msg;
}

TEST(AP_DDS_GPS_INPUT, test_convert_fields)
{
    AP_GPS_DDS::NavSatFix pkt {};
    AP_DDS_GPS_Input::convert(make_msg(), pkt);

    EXPECT_EQ(pkt.gps_week, 2436U);
    EXPECT_EQ(pkt.gps_tow_ms, 129618200U);
    EXPECT_EQ(pkt.fix_type, AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(pkt.num_sats, 14U);

    EXPECT_DOUBLE_EQ(pkt.latitude_deg, 52.5125623);
    EXPECT_DOUBLE_EQ(pkt.longitude_deg, 13.3269871);
    // MSL, not ellipsoid height
    EXPECT_FLOAT_EQ(pkt.altitude_amsl_m, 36.5f);

    // UBX velocity is already NED; no axis swap
    EXPECT_TRUE(pkt.have_velocity);
    EXPECT_TRUE(pkt.have_vertical_velocity);
    EXPECT_FLOAT_EQ(pkt.velocity_ned.x, 1.5f);
    EXPECT_FLOAT_EQ(pkt.velocity_ned.y, -2.0f);
    EXPECT_FLOAT_EQ(pkt.velocity_ned.z, 0.25f);

    EXPECT_TRUE(pkt.have_accuracy);
    EXPECT_FLOAT_EQ(pkt.horizontal_accuracy, 0.8f);
    EXPECT_FLOAT_EQ(pkt.vertical_accuracy, 1.2f);
    EXPECT_FLOAT_EQ(pkt.speed_accuracy, 0.15f);

    EXPECT_TRUE(pkt.have_dop);
    EXPECT_FLOAT_EQ(pkt.hdop, 1.35f);
    EXPECT_FLOAT_EQ(pkt.vdop, 1.35f);
}

TEST(AP_DDS_GPS_INPUT, test_convert_fix_type)
{
    const struct {
        int32_t ubx;
        AP_GPS_FixType ap;
    } cases[] {
        {0, AP_GPS_FixType::NONE},     // no fix
        {1, AP_GPS_FixType::NONE},     // dead reckoning only
        {2, AP_GPS_FixType::FIX_2D},
        {3, AP_GPS_FixType::FIX_3D},
        {4, AP_GPS_FixType::FIX_3D},   // GNSS + dead reckoning
        {5, AP_GPS_FixType::NONE},     // time only
        {-1, AP_GPS_FixType::NONE},
    };
    for (const auto &c : cases) {
        auto msg = make_msg();
        msg.fixtype = c.ubx;
        AP_GPS_DDS::NavSatFix pkt {};
        AP_DDS_GPS_Input::convert(msg, pkt);
        EXPECT_EQ(pkt.fix_type, c.ap) << "UBX fixType " << c.ubx;
    }
}

TEST(AP_DDS_GPS_INPUT, test_convert_edge_values)
{
    auto msg = make_msg();
    msg.numsv = 300;
    msg.year = 0;   // date not valid: week unknown, TOW still carried
    AP_GPS_DDS::NavSatFix pkt {};
    AP_DDS_GPS_Input::convert(msg, pkt);
    EXPECT_EQ(pkt.num_sats, 255U);
    EXPECT_EQ(pkt.gps_week, 0U);
    EXPECT_EQ(pkt.gps_tow_ms, 129618200U);

    msg.numsv = -3;
    AP_DDS_GPS_Input::convert(msg, pkt);
    EXPECT_EQ(pkt.num_sats, 0U);
}

#endif // AP_DDS_GPS_INPUT_SUB_ENABLED

AP_GTEST_MAIN()
