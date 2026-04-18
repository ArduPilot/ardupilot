#include <AP_gtest.h>

#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>
#include <AP_Math/AP_Math.h>

// check if two vector3f are equal 
#define EXPECT_VECTOR3F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
        EXPECT_FLOAT_EQ(v1[2], v2[2]);          \
    } while (false);


TEST(Lines3dTests, ClosestDistBetweenLinePoint)
{
    // check if the 2-d and 3-d variant of this method is same if the third-dimension is zero
    float dist_3d = Vector3f::closest_distance_between_line_and_point(Vector3f{0.0f, 1.0f, 0.0f}, Vector3f{0.0f, 10.0f, 0.0f}, Vector3f{6.0f, 5.0f, 0.0f});
    float dist_2d = Vector2f::closest_distance_between_line_and_point(Vector2f{0.0f, 1.0f}, Vector2f{0.0f, 10.0f}, Vector2f{6.0f, 5.0f});
    EXPECT_FLOAT_EQ(dist_2d, dist_3d);

    // random point test
    const Vector3f intersection = Vector3f::point_on_line_closest_to_other_point(Vector3f{}, Vector3f{0.0f, 10.0f, 10.0f}, Vector3f{0.0f, 5.0f, 5.0f});
    EXPECT_VECTOR3F_EQ((Vector3f{0.0f, 5.0f, 5.0f}), intersection);

    // check protection agains null length
    const Vector3f intersection_null = Vector3f::point_on_line_closest_to_other_point(Vector3f{1.0f, 1.0f, 1.0f}, Vector3f{1.0f, 1.0f, 1.0f}, Vector3f{0.0f, 5.0f, 5.0f});
    EXPECT_VECTOR3F_EQ((Vector3f{1.0f, 1.0f, 1.0f}), intersection_null);
}

TEST(Lines3dTests, SegmentToSegmentCloestPoint)
{
    // random segments test: X-axis seg vs Y-axis-at-z=1 seg → closest on seg2 is (0,0,1)
    Vector3f intersection;
    Vector3f::segment_to_segment_closest_point(Vector3f{-10.0f,0.0f,0.0f}, Vector3f{10.0f,0.0f,0.0f}, Vector3f{0.0f, -5.0f, 1.0}, Vector3f{0.0f, 5.0f, 1.0f}, intersection);
    EXPECT_NEAR(intersection.x, 0.0f, 1e-5f);
    EXPECT_NEAR(intersection.y, 0.0f, 1e-5f);
    EXPECT_NEAR(intersection.z, 1.0f, 1e-5f);

    // check for intersecting segments. Verify with the 2-d variant
    Vector3f::segment_to_segment_closest_point(Vector3f{}, Vector3f{10.0f,10.0f,0.0f}, Vector3f{2.0f, -10.0f, 0.0}, Vector3f{3.0f, 10.0f, 0.0f}, intersection);
    Vector2f intersection_2d;
    const bool result = Vector2f::segment_intersection(Vector2f{}, Vector2f{10.0f,10.0}, Vector2f{2.0f, -10.0f}, Vector2f{3.0f, 10.0f}, intersection_2d);
    EXPECT_EQ(true, result);
    EXPECT_NEAR(intersection.x, intersection_2d.x, 1e-4f);
    EXPECT_NEAR(intersection.y, intersection_2d.y, 1e-4f);
    EXPECT_NEAR(intersection.z, 0.0f, 1e-5f);
}

TEST(Lines3dTests, SegmentToSegmentEdgeCases)
{
    Vector3f cp;

    // T-intersection: horizontal seg touches vertical seg at one endpoint
    // seg1: (0,-5,0)→(0,5,0)  seg2: (0,0,0)→(5,0,0)
    // closest point on seg2 from seg1 is (0,0,0)
    Vector3f::segment_to_segment_closest_point(
        Vector3f{0,-5,0}, Vector3f{0,5,0},
        Vector3f{0,0,0},  Vector3f{5,0,0},
        cp);
    EXPECT_NEAR(cp.x, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.y, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.z, 0.0f, 1e-5f);

    // Parallel segments at different heights: closest is the nearest endpoint pair
    // seg1: (0,0,0)→(10,0,0)  seg2: (0,0,2)→(10,0,2)  (parallel, z=2 offset)
    // Starts are closest; result is seg2_start=(0,0,2)
    Vector3f::segment_to_segment_closest_point(
        Vector3f{0,0,0}, Vector3f{10,0,0},
        Vector3f{0,0,2}, Vector3f{10,0,2},
        cp);
    EXPECT_NEAR(cp.x, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.y, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.z, 2.0f, 1e-5f);

    // Touching endpoints: seg1 end == seg2 start
    // seg1: (0,0,0)→(1,0,0)  seg2: (1,0,0)→(2,0,0)
    // closest point on seg2 from seg1 is the shared point (1,0,0)
    Vector3f::segment_to_segment_closest_point(
        Vector3f{0,0,0}, Vector3f{1,0,0},
        Vector3f{1,0,0}, Vector3f{2,0,0},
        cp);
    EXPECT_NEAR(cp.x, 1.0f, 1e-5f);
    EXPECT_NEAR(cp.y, 0.0f, 1e-5f);
    EXPECT_NEAR(cp.z, 0.0f, 1e-5f);

    // Identical segments: degenerate parallel case; result is seg2_start=(1,2,3)
    Vector3f::segment_to_segment_closest_point(
        Vector3f{1,2,3}, Vector3f{4,5,6},
        Vector3f{1,2,3}, Vector3f{4,5,6},
        cp);
    EXPECT_NEAR(cp.x, 1.0f, 1e-5f);
    EXPECT_NEAR(cp.y, 2.0f, 1e-5f);
    EXPECT_NEAR(cp.z, 3.0f, 1e-5f);
}

TEST(Lines3dTests, ClosestDistBetweenLinePointEdgeCases)
{
    // Point coincident with a segment endpoint: distance = 0
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f{0,0,0}, Vector3f{5,0,0}, Vector3f{0,0,0}),
        0.0f, 1e-6f);

    // Point at the other endpoint: distance = 0
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f{0,0,0}, Vector3f{5,0,0}, Vector3f{5,0,0}),
        0.0f, 1e-6f);

    // 3-D diagonal segment and off-axis point (projection within segment)
    // Segment (0,0,0)→(10,10,10), point (10,0,0): foot=(10/3,10/3,10/3); distance=sqrt(600)/3
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f{0,0,0}, Vector3f{10,10,10}, Vector3f{10,0,0}),
        sqrtf(600.0f)/3.0f, 1e-5f);

    // Point beyond segment end: clamped to w2; distance = |p - w2|
    // Segment (0,0,0)→(3,4,0), point (6,8,0) (same direction, beyond)
    // foot = w2 = (3,4,0); distance = |(6,8,0)-(3,4,0)| = |(3,4,0)| = 5
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f{0,0,0}, Vector3f{3,4,0}, Vector3f{6,8,0}),
        5.0f, 1e-5f);
}

AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
