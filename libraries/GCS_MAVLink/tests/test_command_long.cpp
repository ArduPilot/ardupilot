#include <AP_gtest.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

GCS_Dummy _gcs;

class TestGCS : public GCS_MAVLINK_Dummy {
public:
    using GCS_MAVLINK_Dummy::GCS_MAVLINK_Dummy;
    using GCS_MAVLINK::convert_COMMAND_LONG_to_COMMAND_INT;
};

TEST(GCS_MAVLink, ConvertCommandLongLocationValid)
{
    TestGCS dummy{*hal.serial(0)};
    mavlink_command_long_t in{};
    mavlink_command_int_t out{};

    in.command = MAV_CMD_DO_SET_ROI_LOCATION;
    in.param5 = -35.25f; // lat in deg
    in.param6 = 149.5f;  // lon in deg
    in.param7 = 584.0f;  // alt in meters

    EXPECT_TRUE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.command, (uint16_t)MAV_CMD_DO_SET_ROI_LOCATION);
    EXPECT_EQ(out.x, -352500000);
    EXPECT_EQ(out.y, 1495000000);
    EXPECT_FLOAT_EQ(out.z, 584.0f);
}

TEST(GCS_MAVLink, ConvertCommandLongLocationBoundaries)
{
    TestGCS dummy{*hal.serial(0)};
    mavlink_command_long_t in{};
    mavlink_command_int_t out{};

    in.command = MAV_CMD_DO_SET_ROI_LOCATION;

    // Maximum and minimum valid longitude on Earth (+/- 180 deg)
    in.param5 = 180.0f;
    in.param6 = -180.0f;
    EXPECT_TRUE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 1800000000);
    EXPECT_EQ(out.y, -1800000000);

    // Just above maximum coordinate representable in int32 * 1e7
    in.param5 = 215.0f;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));

    // Just below minimum coordinate representable in int32 * 1e7
    in.param5 = -215.0f;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
}

TEST(GCS_MAVLink, ConvertCommandLongLocationOverflow)
{
    TestGCS dummy{*hal.serial(0)};
    mavlink_command_long_t in{};
    mavlink_command_int_t out{};

    in.command = MAV_CMD_DO_SET_ROI_LOCATION;
    // Issue #34854: coordinates sent pre-scaled by 1e7 as float, exceeding INT32_MAX when multiplied by 1e7
    in.param5 = 1491652371.0f;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 0);

    // Coordinate causing scaled float to exceed INT32_MIN
    in.param5 = -1491652371.0f;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 0);

    // Coordinate with infinity
    in.param5 = INFINITY;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 0);

    // Negative infinity
    in.param5 = -INFINITY;
    in.param6 = 0.0f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 0);

    // Latitude param6 (longitude) overflow
    in.param5 = 0.0f;
    in.param6 = 1e12f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.y, 0);
}

TEST(GCS_MAVLink, ConvertCommandLongNaNHandling)
{
    TestGCS dummy{*hal.serial(0)};
    mavlink_command_long_t in{};
    mavlink_command_int_t out{};

    in.command = MAV_CMD_DO_SET_ROI_LOCATION;
    in.param5 = NAN;
    in.param6 = NAN;
    EXPECT_TRUE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 0);
    EXPECT_EQ(out.y, 0);
}

TEST(GCS_MAVLink, ConvertCommandLongNonLocationCommand)
{
    TestGCS dummy{*hal.serial(0)};
    mavlink_command_long_t in{};
    mavlink_command_int_t out{};

    in.command = MAV_CMD_NAV_TAKEOFF; // Non-location in GCS_MAVLINK
    in.param5 = 123.0f;
    in.param6 = 456.0f;
    EXPECT_TRUE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
    EXPECT_EQ(out.x, 123);
    EXPECT_EQ(out.y, 456);

    // Non-location overflow (> INT32_MAX or < INT32_MIN)
    in.param5 = 1e15f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));

    in.param5 = -1e15f;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));

    in.param5 = INFINITY;
    EXPECT_FALSE(dummy.convert_COMMAND_LONG_to_COMMAND_INT(in, out));
}

AP_GTEST_MAIN()
