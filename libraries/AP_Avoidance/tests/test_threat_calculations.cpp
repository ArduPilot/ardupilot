#include <AP_gtest.h>

#include <AP_Avoidance/AP_Avoidance.h>

#define TEST_CLOSEST_APPROACH(loc1_lat,loc1_lon, vel1_x,vel1_y, loc2_lat,loc2_lon, vel2_x,vel2_y, time, expected_distance) \
    do {                                                                \
        Location loc1;                                                  \
        loc1.lat = loc1_lat *1e7;                                       \
        loc1.lng = loc1_lon *1e7;                                       \
        loc1.alt = 0;                                                   \
        Vector3f vel1;                                                  \
        vel1.x = vel1_x;                                                \
        vel1.y = vel1_y;                                                \
        vel1.z = 0;                                                     \
        Location loc2;                                                  \
        loc2.lat = loc2_lat *1e7;                                       \
        loc2.lng = loc2_lon *1e7;                                       \
        loc2.alt = 0;                                                   \
        Vector3f vel2;                                                  \
        vel2.x = vel2_x;                                                \
        vel2.y = vel2_y;                                                \
        vel2.z = 0;                                                     \
        float result = closest_approach_xy(loc1, vel1, loc2, vel2, time); \
        EXPECT_FLOAT_EQ(result, expected_distance);                     \
    } while (0)


TEST(ThreatTests, Distance)
{

    // nb: 0.0001f degrees lat/lon is ~15 metres
    //                    Loc1        Vel1       Loc2              Vel2          dT    result
    TEST_CLOSEST_APPROACH(0.0,0.0,    0.0,0.0,  0.0001f,0.0001f,  0.0,0.0,      30,   15.727118);

    TEST_CLOSEST_APPROACH(0.0,0.0,    0.0,-10.0,  0.0001f,0.0001f,  0.0,0.0,      30,   15.727118);
    TEST_CLOSEST_APPROACH(0.0,0.0,    0.0,10.0,  0.0001f,0.0001f,  0.0,0.0,      30,   11.120752);

    TEST_CLOSEST_APPROACH(0.0,0.0,    0.0,1.0,  0.0001f,0.0001f,  0.0,0.0,      30,   11.120752);

    TEST_CLOSEST_APPROACH(0.0,-0.1,  0.0,1.0,  0.0001f,0.0001f,  0.0,0.0,      30,   11113.011);

    TEST_CLOSEST_APPROACH(0.1,0.10001f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(0.1,0.10002f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(0.1,0.10004f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(0.1,0.10008f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(0.1,0.10010f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(0.1,0.10011f,  0.0,1.0,  0.1001f,0.1001f,  0.0,0.0,      30,   11.187406);

    TEST_CLOSEST_APPROACH(1.0,1.00001f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(1.0,1.00002f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(1.0,1.00004f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(1.0,1.00008f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(1.0,1.00010f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.131885);
    TEST_CLOSEST_APPROACH(1.0,1.00011f,  0.0,1.0,  1.0001f,1.0001f,  0.0,0.0,      30,   11.187388);

    TEST_CLOSEST_APPROACH(45.0,45.00001f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.031697);
    TEST_CLOSEST_APPROACH(45.0,45.00002f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.031697);
    TEST_CLOSEST_APPROACH(45.0,45.00004f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.031697);
    TEST_CLOSEST_APPROACH(45.0,45.00008f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.031697);
    TEST_CLOSEST_APPROACH(45.0,45.00010f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.031697);
    TEST_CLOSEST_APPROACH(45.0,45.00011f,  0.0,1.0,  45.0001f,45.0001f,  0.0,0.0,      30,   11.068774);
}


AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
