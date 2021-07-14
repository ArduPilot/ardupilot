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
    EXPECT_VECTOR3F_EQ((Vector3f{0.0f, 0.0f, 0.0f}), intersection_null);
}

TEST(Lines3dTests, SegmentToSegmentCloestPoint)
{
    // random segments test
    Vector3f intersection;
    Vector3f::segment_to_segment_closest_point(Vector3f{-10.0f,0.0f,0.0f}, Vector3f{10.0f,0.0f,0.0f}, Vector3f{0.0f, -5.0f, 1.0}, Vector3f{0.0f, 5.0f, 1.0f}, intersection);
    EXPECT_VECTOR3F_EQ(intersection, (Vector3f{0.0f, 0.0f, 1.0f}));

    // check for intersecting segments. Verify with the 2-d variant
    Vector3f::segment_to_segment_closest_point(Vector3f{}, Vector3f{10.0f,10.0f,0.0f}, Vector3f{2.0f, -10.0f, 0.0}, Vector3f{3.0f, 10.0f, 0.0f}, intersection);
    Vector2f intersection_2d;
    const bool result = Vector2f::segment_intersection(Vector2f{}, Vector2f{10.0f,10.0}, Vector2f{2.0f, -10.0f}, Vector2f{3.0f, 10.0f}, intersection_2d);
    EXPECT_EQ(true, result);
    EXPECT_VECTOR3F_EQ(intersection, (Vector3f(intersection_2d.x, intersection_2d.y, 0.0f)));
}

AP_GTEST_MAIN()
int hal = 0; //weirdly the build will fail without this
