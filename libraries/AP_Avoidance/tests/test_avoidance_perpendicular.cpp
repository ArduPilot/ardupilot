#include <AP_gtest.h>

#include <AP_Avoidance/AvoidanceHandler.h>

#define SQRT_2 0.70710677

#define EXPECT_VECTOR3F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
        EXPECT_FLOAT_EQ(v1[2], v2[2]);          \
    } while (false);

// note this function translates the velocity from XYZ to NED!
#define TEST_PERPENDICULAR_XYZ(loc1_lat,loc1_lon,loc1_alt, vel1_x,vel1_y,vel1_z, loc2_lat,loc2_lon,loc2_alt, expected_x,expected_y,expected_z) \
    do {                                                                \
        Location loc1;                                                  \
        loc1.lat = loc1_lat *1e7;                                       \
        loc1.lng = loc1_lon *1e7;                                       \
        loc1.alt = loc1_alt;                                                   \
        Vector3f vel1;                                                  \
        vel1.x = vel1_y;                                                \
        vel1.y = vel1_x;                                                \
        vel1.z = -vel1_z;                                               \
        Location loc2;                                                  \
        loc2.lat = loc2_lat *1e7;                                       \
        loc2.lng = loc2_lon *1e7;                                       \
        loc2.alt = loc2_alt;                                                   \
        Vector3f expected=Vector3f(expected_x,expected_y,expected_z);   \
        Vector3f result;                                                \
        result = AvoidanceHandler::perpendicular_xyz(loc1, vel1, loc2); \
        result.normalize();                                             \
        EXPECT_VECTOR3F_EQ(expected, result);                           \
    } while (0)


TEST(ThreatTests, Distance)
{

    // nb: 0.0001f degrees lat/lon is ~15 metres
    //                    Loc1(LatLonAlt)        Vel1(XYZ)       Loc2              expected(XYZ)

    TEST_PERPENDICULAR_XYZ(0,0,0, 0,0,0,  0.0001f,0.0001f,0, SQRT_2,SQRT_2,0.0); // no velocity means we get garbage back
    TEST_PERPENDICULAR_XYZ(0,0,0, -10.0,0,0,  0.0001f,0.0001f,0,  0,1.0,0.0);
    TEST_PERPENDICULAR_XYZ(0.0001f,0.0001f,0,    -10.0,0,0,  0,0,0,  0,-1,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,    10.0,0,0,  0.0001f,0.0001f,0,  0,1.0,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,    1.0,0,0,  0.0001f,0.0001f,0,  0,1.0,0.0);
    TEST_PERPENDICULAR_XYZ(0,-0.1,0,  1.0,0,0,  0.0001f,0.0001f,0,  0,1.0,0.0);
    TEST_PERPENDICULAR_XYZ(0.1,0.10001f,0,  1.0,0,0,  0.1001f,0.1001f,0,  0,1.0,0.0);

    TEST_PERPENDICULAR_XYZ(0,0,0,  0,1.0,0,  0.0001f,0.0001f,0,  1.0,0,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  1.0,1.0,0,  0.0000f,0.0001f,0,  SQRT_2,-SQRT_2,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  2.0,2.0,0,  0.0000f,0.0001f,0,  SQRT_2,-SQRT_2,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  1.0,1.0,0,  0.0000f,-0.0001f,0,  -SQRT_2,SQRT_2,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  2.0,2.0,0,  0.0000f,-0.0001f,0,  -SQRT_2,SQRT_2,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  1.0,2.0,0,  0.0000f,0.0001f,0,  0.89442718,-0.44721359,0.0);
    TEST_PERPENDICULAR_XYZ(0,0,0,  2.0,1.0,0,  0.0000f,0.0001f,0,  0.44721365,-0.89442718,0.0);
    TEST_PERPENDICULAR_XYZ(0,-0.0001,0,  2.0,1.0,0,  0,0,0,  0.44721365,-0.89442718,0.0);

// TEST_PERPENDICULAR_XYZ(0.1,0.10002f,  1.0,0.0,0.0,  0.1001f,0.1001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(0.1,0.10004f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(0.1,0.10008f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(0.1,0.10010f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(0.1,0.10011f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,0.0);

    // TEST_PERPENDICULAR_XYZ(1.0,1.00001f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(1.0,1.00002f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(1.0,1.00004f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(1.0,1.00008f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(1.0,1.00010f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(1.0,1.00011f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,0.0);

    // TEST_PERPENDICULAR_XYZ(45.0,45.00001f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(45.0,45.00002f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(45.0,45.00004f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(45.0,45.00008f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(45.0,45.00010f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
    // TEST_PERPENDICULAR_XYZ(45.0,45.00011f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,0.0);
}


AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
