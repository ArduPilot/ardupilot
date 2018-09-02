#include <AP_gtest.h>

#include <AP_Math/vector2.h>

#define EXPECT_VECTOR2F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
    } while (false);

#define TEST_INTERSECT(s1p1x,s1p1y,s1p2x,s1p2y, s2p1x,s2p1y,s2p2x,s2p2y, ix,iy, expected) \
    do {                                                                \
        const Vector2f s1p1(s1p1x,s1p1y);                               \
        const Vector2f s1p2(s1p2x,s1p2y);                               \
        const Vector2f s2p1(s2p1x,s2p1y);                               \
        const Vector2f s2p2(s2p2x,s2p2y);                               \
        Vector2f calculated_intersection;                               \
        bool result = Vector2f::segment_intersection(s1p1, s1p2, s2p1, s2p2, calculated_intersection); \
        EXPECT_EQ(expected, result);                                    \
        if (result == expected && expected) {                           \
            Vector2f expected_intersection{ix, iy};                     \
            EXPECT_VECTOR2F_EQ(calculated_intersection, expected_intersection); \
        }                                                               \
        result = Vector2f::segment_intersection(s2p1, s2p2, s1p1, s1p2, calculated_intersection); \
        EXPECT_EQ(expected, result);                                    \
        if (expected) {                           \
            Vector2f expected_intersection{ix, iy};                     \
            EXPECT_VECTOR2F_EQ(calculated_intersection, expected_intersection); \
            EXPECT_EQ(Vector2f::point_on_segment(calculated_intersection,s1p1, s1p2), true); \
            EXPECT_EQ(Vector2f::point_on_segment(calculated_intersection,s2p1, s2p2), true); \
        }                                                               \
    } while (false);

#define SHOULD_INTERSECT(s1p1x,s1p1y,s1p2x,s1p2y, s2p1x,s2p1y,s2p2x,s2p2y, ix, iy) \
    TEST_INTERSECT(s1p1x,s1p1y,s1p2x,s1p2y, s2p1x,s2p1y,s2p2x,s2p2y, ix, iy, true)

#define SHOULD_NOT_INTERSECT(s1p1x,s1p1y,s1p2x,s1p2y, s2p1x,s2p1y,s2p2x,s2p2y) \
    TEST_INTERSECT(s1p1x,s1p1y,s1p2x,s1p2y, s2p1x,s2p1y,s2p2x,s2p2y, 0, 0, false)

TEST(SegmentIntersectionTests, Simple)
{
    SHOULD_INTERSECT(0,0,2,2, 2,0,0,2, 1,1);
    SHOULD_INTERSECT(0,0,5,5, 2,0,0,2, 1,1);
}
TEST(SegmentIntersectionTests, Parallel)
{
    SHOULD_NOT_INTERSECT(0,0,0,1, 2,0,2,1);
}
// the following should probably intersect but don't:
// TEST(SegmentIntersectionTests, Subsegment)
// {
//     SHOULD_INTERSECT(0,0,0,2, 0,0,0,1, 1,1);
// }
// TEST(SegmentIntersectionTests, Identical)
// {
//     SHOULD_INTERSECT(0,0,0,2, 0,0,0,2, 1,1);
// }
TEST(SegmentIntersectionTests, NonIntersecting)
{
    SHOULD_NOT_INTERSECT(0,0,0,2, 1,1,1,2);
    SHOULD_NOT_INTERSECT(0,0,0,1, -2,2,2,2)
}


AP_GTEST_MAIN()
