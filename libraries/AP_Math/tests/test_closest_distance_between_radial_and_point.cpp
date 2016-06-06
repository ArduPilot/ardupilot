#include <AP_gtest.h>

#include <AP_Math/vector2.h>

#define TEST_DISTANCE_BOTH(line_segment_x,line_segment_y, point_x, point_y, expected_length) \
    do {                                                                \
        {                                                               \
            Vector2f line_segment = Vector2f(line_segment_x, line_segment_y); \
            Vector2f point = Vector2f(point_x, point_y);                \
            float result = Vector2<float>::closest_distance_between_radial_and_point( \
                line_segment,                                           \
                point                                                   \
                );                                                      \
            EXPECT_FLOAT_EQ(result, expected_length);                   \
        }                                                               \
    } while (false)


TEST(ThreatTests, Distance)
{

    TEST_DISTANCE_BOTH( 0, 0,  0, 0, 0);
    TEST_DISTANCE_BOTH( 0, 0,  0, 1, 1);

    TEST_DISTANCE_BOTH( 1, 1,  1, 0, sqrt(0.5));
    TEST_DISTANCE_BOTH(-1,-1, -1, 0, sqrt(0.5));

    TEST_DISTANCE_BOTH( 3, 1,  3, 0, 0.94868332);
    TEST_DISTANCE_BOTH( 1, 3,  0, 3, 0.94868332);
    TEST_DISTANCE_BOTH(-1, 3,  0, 3, 0.94868332);
    TEST_DISTANCE_BOTH( 1,-3,  0,-3, 0.94868332);
    TEST_DISTANCE_BOTH(-1,-3,  0,-3, 0.94868332);

    TEST_DISTANCE_BOTH( 2, 2,  1, 1, 0.0);
    TEST_DISTANCE_BOTH( 2, 2,  3, 3, sqrt(2));
    TEST_DISTANCE_BOTH( 2, 2,  2, 2, 0);
    TEST_DISTANCE_BOTH( 0, 0,  1, 1, sqrt(2));
    TEST_DISTANCE_BOTH( 0, 0,  1, 1, sqrt(2));

    TEST_DISTANCE_BOTH( 0, 0,  1, 1, sqrt(2));
    TEST_DISTANCE_BOTH( 1, 1,  0, 3, sqrt(5));
}

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.


