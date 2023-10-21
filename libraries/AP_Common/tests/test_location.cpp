#include <AP_gtest.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyVehicle {
public:
    bool start_cmd(const AP_Mission::Mission_Command& cmd) { return true; };
    bool verify_cmd(const AP_Mission::Mission_Command& cmd) { return true; };
    void mission_complete() { };
    AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&DummyVehicle::start_cmd, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&DummyVehicle::verify_cmd, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&DummyVehicle::mission_complete, void)};
    AP_Terrain terrain;
};

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

static DummyVehicle vehicle;

#define EXPECT_VECTOR2F_EQ(v1, v2)              \
do {                                        \
EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
} while (false);

#define EXPECT_VECTOR3F_EQ(v1, v2)              \
do {                                        \
EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
EXPECT_FLOAT_EQ(v1[2], v2[2]);          \
} while (false);

#define EXPECT_VECTOR2F_NEAR(v1, v2, acc)              \
do {                                        \
EXPECT_NEAR(v1[0], v2[0], acc);          \
EXPECT_NEAR(v1[1], v2[1], acc);          \
} while (false);

#define EXPECT_VECTOR3F_NEAR(v1, v2, acc)              \
do {                                        \
EXPECT_NEAR(v1[0], v2[0], acc);          \
EXPECT_NEAR(v1[1], v2[1], acc);          \
EXPECT_NEAR(v1[2], v2[2], acc);          \
} while (false);

TEST(Location, LatLngWrapping)
{
    struct {
        int32_t start_lat;
        int32_t start_lng;
        Vector2f delta_metres_ne;
        int32_t expected_lat;
        int32_t expected_lng;
    } tests[] {
        {519634000, 1797560000, Vector2f{0, 100000}, 519634000, -1787860777}
    };

    for (auto &test : tests) {
        // forward
        {
            Location loc{test.start_lat, test.start_lng, 0, Location::AltFrame::ABOVE_HOME};
            loc.offset(test.delta_metres_ne[0], test.delta_metres_ne[1]);
            EXPECT_EQ(test.expected_lat, loc.lat);
            EXPECT_EQ(test.expected_lng, loc.lng);
            EXPECT_EQ(0, loc.alt);
        }
        // and now reverse
        {
            Location rev{test.expected_lat, test.expected_lng, 0, Location::AltFrame::ABOVE_HOME};
            rev.offset(-test.delta_metres_ne[0], -test.delta_metres_ne[1]);
            EXPECT_EQ(rev.lat, test.start_lat);
            EXPECT_EQ(rev.lng, test.start_lng);
            EXPECT_EQ(0, rev.alt);
        }
    }
}

TEST(Location, LocOffsetDouble)
{
    struct {
        int32_t home_lat;
        int32_t home_lng;
        Vector2d delta_metres_ne1;
        Vector2d delta_metres_ne2;
        Vector2d expected_pos_change;
    } tests[] {
               -353632620, 1491652373,
               Vector2d{4682795.4576701336, 5953662.7673837934},
               Vector2d{4682797.1904749088, 5953664.1586009059},
               Vector2d{1.7365739,1.4261966},
    };

    for (auto &test : tests) {
        Location home{test.home_lat, test.home_lng, 0, Location::AltFrame::ABOVE_HOME};
        Location loc1 = home;
        Location loc2 = home;
        loc1.offset(test.delta_metres_ne1.x, test.delta_metres_ne1.y);
        loc2.offset(test.delta_metres_ne2.x, test.delta_metres_ne2.y);
        Vector2d diff = loc1.get_distance_NE_double(loc2);
        EXPECT_FLOAT_EQ(diff.x, test.expected_pos_change.x);
        EXPECT_FLOAT_EQ(diff.y, test.expected_pos_change.y);
    }
}

TEST(Location, LocOffset3DDouble)
{
    Location loc {
        -353632620, 1491652373, 60000, Location::AltFrame::ABSOLUTE
    };
    // this is ned, so our latitude should change, and our new
    // location should be above the original:
    loc.offset(Vector3d{1000, 0, -10});
    EXPECT_EQ(loc.lat, -353542788);
    EXPECT_EQ(loc.lng, 1491652373);
    EXPECT_EQ(loc.alt, 61000);
}

TEST(Location, Tests)
{
    Location test_location;
    EXPECT_TRUE(test_location.is_zero());
    EXPECT_FALSE(test_location.initialised());
    const Location test_home{-35362938, 149165085, 100, Location::AltFrame::ABSOLUTE};
    EXPECT_EQ(-35362938, test_home.lat);
    EXPECT_EQ(149165085, test_home.lng);
    EXPECT_EQ(100, test_home.alt);
    EXPECT_EQ(0, test_home.relative_alt);
    EXPECT_EQ(0, test_home.terrain_alt);
    EXPECT_EQ(0, test_home.origin_alt);
    EXPECT_EQ(0, test_home.loiter_ccw);
    EXPECT_EQ(0, test_home.loiter_xtrack);
    EXPECT_TRUE(test_home.initialised());

    const Vector3f test_vect{-42, 42, 0};
    Location test_location3{test_vect, Location::AltFrame::ABOVE_HOME};
    EXPECT_EQ(0, test_location3.lat);
    EXPECT_EQ(0, test_location3.lng);
    EXPECT_EQ(0, test_location3.alt);
    EXPECT_EQ(1, test_location3.relative_alt);
    EXPECT_EQ(0, test_location3.terrain_alt);
    EXPECT_EQ(0, test_location3.origin_alt);
    EXPECT_EQ(0, test_location3.loiter_ccw);
    EXPECT_EQ(0, test_location3.loiter_xtrack);
    EXPECT_FALSE(test_location3.initialised());
    // EXPECT_EXIT(test_location3.change_alt_frame(Location::AltFrame::ABSOLUTE), PANIC something); // TODO check PANIC

    test_location3.set_alt_cm(-420, Location::AltFrame::ABSOLUTE);
    EXPECT_EQ(-420, test_location3.alt);
    EXPECT_EQ(0, test_location3.relative_alt);
    EXPECT_EQ(0, test_location3.terrain_alt);
    EXPECT_EQ(0, test_location3.origin_alt);
    EXPECT_EQ(Location::AltFrame::ABSOLUTE, test_location3.get_alt_frame());

    test_location3.set_alt_cm(420, Location::AltFrame::ABOVE_HOME);
    EXPECT_EQ(420, test_location3.alt);
    EXPECT_EQ(1, test_location3.relative_alt);
    EXPECT_EQ(0, test_location3.terrain_alt);
    EXPECT_EQ(0, test_location3.origin_alt);
    EXPECT_EQ(Location::AltFrame::ABOVE_HOME, test_location3.get_alt_frame());

    test_location3.set_alt_cm(-420, Location::AltFrame::ABOVE_ORIGIN);
    EXPECT_EQ(-420, test_location3.alt);
    EXPECT_EQ(0, test_location3.relative_alt);
    EXPECT_EQ(0, test_location3.terrain_alt);
    EXPECT_EQ(1, test_location3.origin_alt);
    EXPECT_EQ(Location::AltFrame::ABOVE_ORIGIN, test_location3.get_alt_frame());

    test_location3.set_alt_cm(420, Location::AltFrame::ABOVE_TERRAIN);
    EXPECT_EQ(420, test_location3.alt);
    EXPECT_EQ(1, test_location3.relative_alt);
    EXPECT_EQ(1, test_location3.terrain_alt);
    EXPECT_EQ(0, test_location3.origin_alt);
    EXPECT_EQ(Location::AltFrame::ABOVE_TERRAIN, test_location3.get_alt_frame());

    // No TERRAIN, NO HOME, NO ORIGIN
    AP::terrain()->set_enabled(false);
    for (auto current_frame = Location::AltFrame::ABSOLUTE;
         current_frame <= Location::AltFrame::ABOVE_TERRAIN;
         current_frame = static_cast<Location::AltFrame>(
                 (uint8_t) current_frame + 1)) {
        for (auto desired_frame = Location::AltFrame::ABSOLUTE;
             desired_frame <= Location::AltFrame::ABOVE_TERRAIN;
             desired_frame = static_cast<Location::AltFrame>(
                     (uint8_t) desired_frame + 1)) {
            test_location3.set_alt_cm(420, current_frame);
            if (current_frame == desired_frame) {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
                continue;
            }
            if (current_frame == Location::AltFrame::ABOVE_TERRAIN
                    || desired_frame == Location::AltFrame::ABOVE_TERRAIN) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else if (current_frame == Location::AltFrame::ABOVE_ORIGIN
                    || desired_frame == Location::AltFrame::ABOVE_ORIGIN) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else if (current_frame == Location::AltFrame::ABOVE_HOME
                    || desired_frame == Location::AltFrame::ABOVE_HOME) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
            }
        }
    }
    // NO TERRAIN, NO ORIGIN
    EXPECT_TRUE(vehicle.ahrs.set_home(test_home));
    for (auto current_frame = Location::AltFrame::ABSOLUTE;
         current_frame <= Location::AltFrame::ABOVE_TERRAIN;
         current_frame = static_cast<Location::AltFrame>(
                 (uint8_t) current_frame + 1)) {
        for (auto desired_frame = Location::AltFrame::ABSOLUTE;
             desired_frame <= Location::AltFrame::ABOVE_TERRAIN;
             desired_frame = static_cast<Location::AltFrame>(
                     (uint8_t) desired_frame + 1)) {
            test_location3.set_alt_cm(420, current_frame);
            if (current_frame == desired_frame) {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
                continue;
            }
            if (current_frame == Location::AltFrame::ABOVE_TERRAIN
                    || desired_frame == Location::AltFrame::ABOVE_TERRAIN) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else if (current_frame == Location::AltFrame::ABOVE_ORIGIN
                    || desired_frame == Location::AltFrame::ABOVE_ORIGIN) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
            }

        }
    }
    // NO Origin
    AP::terrain()->set_enabled(true);
    for (auto current_frame = Location::AltFrame::ABSOLUTE;
         current_frame <= Location::AltFrame::ABOVE_TERRAIN;
         current_frame = static_cast<Location::AltFrame>(
                 (uint8_t) current_frame + 1)) {
        for (auto desired_frame = Location::AltFrame::ABSOLUTE;
             desired_frame <= Location::AltFrame::ABOVE_TERRAIN;
             desired_frame = static_cast<Location::AltFrame>(
                     (uint8_t) desired_frame + 1)) {
            test_location3.set_alt_cm(420, current_frame);
            if (current_frame == desired_frame) {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
                continue;
            }
            if (current_frame == Location::AltFrame::ABOVE_ORIGIN
                    || desired_frame == Location::AltFrame::ABOVE_ORIGIN) {
                EXPECT_FALSE(test_location3.change_alt_frame(desired_frame));
            } else {
                EXPECT_TRUE(test_location3.change_alt_frame(desired_frame));
            }
        }
    }

    Vector2f test_vec2;
    EXPECT_FALSE(test_home.get_vector_xy_from_origin_NE(test_vec2));
    Vector3f test_vec3;
    EXPECT_FALSE(test_home.get_vector_from_origin_NEU(test_vec3));

    Location test_origin = test_home;
    test_origin.offset(2, 2);
    const Vector3f test_vecto{200, 200, 10};
    const Location test_location4{test_vecto, Location::AltFrame::ABOVE_ORIGIN};
    EXPECT_EQ(10, test_location4.alt);
    EXPECT_EQ(0, test_location4.relative_alt);
    EXPECT_EQ(0, test_location4.terrain_alt);
    EXPECT_EQ(1, test_location4.origin_alt);
    EXPECT_EQ(0, test_location4.loiter_ccw);
    EXPECT_EQ(0, test_location4.loiter_xtrack);
    EXPECT_TRUE(test_location4.initialised());

    const Location test_location_empty{test_vect, Location::AltFrame::ABOVE_HOME};
    EXPECT_FALSE(test_location_empty.get_vector_from_origin_NEU(test_vec3));
}

TEST(Location, Distance)
{
    const Location test_home{-35362938, 149165085, 100, Location::AltFrame::ABSOLUTE};
    const Location test_home2{-35363938, 149165085, 100, Location::AltFrame::ABSOLUTE};
    EXPECT_FLOAT_EQ(11.131885, test_home.get_distance(test_home2));
    EXPECT_FLOAT_EQ(0, test_home.get_distance(test_home));
    EXPECT_VECTOR2F_EQ(Vector2f(0, 0), test_home.get_distance_NE(test_home));
    EXPECT_VECTOR2F_EQ(Vector2f(-11.131885, 0), test_home.get_distance_NE(test_home2));
    EXPECT_VECTOR2F_EQ(Vector3f(0, 0, 0), test_home.get_distance_NED(test_home));
    EXPECT_VECTOR2F_EQ(Vector3f(-11.131885, 0, 0), test_home.get_distance_NED(test_home2));
    Location test_loc = test_home;
    test_loc.offset(-11.131886, 0);
    EXPECT_TRUE(test_loc.same_latlon_as(test_home2));
    test_loc = test_home;
    test_loc.offset(-11.131885, 0);
    test_loc.offset_bearing(0, 11.131885);
    EXPECT_TRUE(test_loc.same_latlon_as(test_home));

    test_loc.offset_bearing_and_pitch(0, 2, -11.14);
    EXPECT_TRUE(test_loc.same_latlon_as(test_home2));
    EXPECT_EQ(62, test_loc.alt);

    test_loc = Location(-35362633, 149165085, 0, Location::AltFrame::ABOVE_HOME);
    int32_t bearing = test_home.get_bearing_to(test_loc);
    EXPECT_EQ(0, bearing);

    test_loc = Location(-35363711, 149165085, 0, Location::AltFrame::ABOVE_HOME);
    bearing = test_home.get_bearing_to(test_loc);
    EXPECT_EQ(18000, bearing);

    test_loc = Location(-35362938, 149166085, 0, Location::AltFrame::ABOVE_HOME);
    bearing = test_home.get_bearing_to(test_loc);
    EXPECT_EQ(9000, bearing);

    test_loc = Location(-35362938, 149164085, 0, Location::AltFrame::ABOVE_HOME);
    bearing = test_home.get_bearing_to(test_loc);
    EXPECT_EQ(27000, bearing);

    test_loc = Location(-35361938, 149164085, 0, Location::AltFrame::ABOVE_HOME);
    bearing = test_home.get_bearing_to(test_loc);
    EXPECT_EQ(31503, bearing);
    const float bearing_rad = test_home.get_bearing(test_loc);
    EXPECT_FLOAT_EQ(5.4982867, bearing_rad);

}

TEST(Location, Sanitize)
{
    // we will sanitize test_loc with test_default_loc
    // test_home is just for reference
    const Location test_home{-35362938, 149165085, 100, Location::AltFrame::ABSOLUTE};
    EXPECT_TRUE(vehicle.ahrs.set_home(test_home));
    const Location test_default_loc{-35362938, 149165085, 200, Location::AltFrame::ABSOLUTE};
    Location test_loc;
    test_loc.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
    EXPECT_TRUE(test_loc.sanitize(test_default_loc));
    EXPECT_TRUE(test_loc.same_latlon_as(test_default_loc));
    int32_t default_loc_alt;
    // we should compare test_loc alt and test_default_loc alt in same frame , in this case, ABOVE HOME
    EXPECT_TRUE(test_default_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, default_loc_alt));
    EXPECT_EQ(test_loc.alt, default_loc_alt);
    test_loc = Location(91*1e7, 0, 0, Location::AltFrame::ABSOLUTE);
    EXPECT_TRUE(test_loc.sanitize(test_default_loc));
    EXPECT_TRUE(test_loc.same_latlon_as(test_default_loc));
    EXPECT_NE(test_default_loc.alt, test_loc.alt);
    test_loc = Location(0, 181*1e7, 0, Location::AltFrame::ABSOLUTE);
    EXPECT_TRUE(test_loc.sanitize(test_default_loc));
    EXPECT_TRUE(test_loc.same_latlon_as(test_default_loc));
    EXPECT_NE(test_default_loc.alt, test_loc.alt);
    test_loc = Location(42*1e7, 42*1e7, 420, Location::AltFrame::ABSOLUTE);
    EXPECT_FALSE(test_loc.sanitize(test_default_loc));
    EXPECT_FALSE(test_loc.same_latlon_as(test_default_loc));
    EXPECT_NE(test_default_loc.alt, test_loc.alt);
}

TEST(Location, Line)
{
    const Location test_home{35362938, 149165085, 100, Location::AltFrame::ABSOLUTE};
    const Location test_wp_last{35362960, 149165085, 100, Location::AltFrame::ABSOLUTE};
    Location test_wp{35362940, 149165085, 100, Location::AltFrame::ABSOLUTE};
    EXPECT_FALSE(test_wp.past_interval_finish_line(test_home, test_wp_last));
    EXPECT_TRUE(test_wp.past_interval_finish_line(test_home, test_home));
    test_wp.lat = 35362970;
    EXPECT_TRUE(test_wp.past_interval_finish_line(test_home, test_wp_last));
}

/*
  check if we obey basic euclidean geometry rules of position
  addition/subtraction
 */
TEST(Location, OffsetError)
{
    // test at 10km from origin
    const float ofs_ne = 10e3 / sqrtf(2.0);
    for (float lat = -80; lat <= 80; lat += 10.0) {
        Location origin{int32_t(lat*1e7), 0, 0, Location::AltFrame::ABOVE_HOME};
        Location loc = origin;
        loc.offset(ofs_ne, ofs_ne);
        Location loc2 = loc;
        loc2.offset(-ofs_ne, -ofs_ne);
        float dist = origin.get_distance(loc2);
        EXPECT_FLOAT_EQ(dist, 0);
    }
}

AP_GTEST_MAIN()
