#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(Vector3Test, Operator)
{
    Vector3f v_float0{1.0f, 1.0f,1.0f};
    EXPECT_FALSE(v_float0.is_zero());
    v_float0 = Vector3f();
    EXPECT_TRUE(v_float0.is_zero());
    v_float0[1] = 1.0f;
    EXPECT_FALSE(v_float0.is_zero());
    const float testf1 = v_float0[1];
    EXPECT_TRUE(is_equal(testf1, 1.0f));
    v_float0.zero();
    EXPECT_TRUE(v_float0.is_zero());

    Vector3i v_inti1{1, 1, 1};
    EXPECT_FALSE(v_inti1.is_zero());
    v_inti1 = Vector3i();
    EXPECT_TRUE(v_inti1.is_zero());
    v_inti1[0] = 1;
    EXPECT_FALSE(v_inti1.is_zero());
    const int16_t testi1 = v_inti1[0];
    EXPECT_TRUE(1 == testi1);
    v_inti1.zero();
    EXPECT_TRUE(v_inti1.is_zero());

    Vector3ui v_uinti1{1u, 1u, 1u};
    EXPECT_FALSE(v_uinti1.is_zero());
    v_uinti1 = Vector3ui();
    EXPECT_TRUE(v_uinti1.is_zero());
    v_uinti1[0] = 1u;
    EXPECT_FALSE(v_uinti1.is_zero());
    const uint16_t testui1 = v_uinti1[0];
    EXPECT_TRUE(1u == testui1);
    v_uinti1.zero();
    EXPECT_TRUE(v_uinti1.is_zero());

    Vector3l v_intl1{1, 1, 1};
    EXPECT_FALSE(v_intl1.is_zero());
    v_intl1 = Vector3l();
    EXPECT_TRUE(v_intl1.is_zero());
    v_intl1[0] = 1;
    EXPECT_FALSE(v_intl1.is_zero());
    const int32_t testl1 = v_intl1[0];
    EXPECT_TRUE(1 == testl1);
    v_intl1.zero();
    EXPECT_TRUE(v_intl1.is_zero());

    Vector3ul v_uint1l{1, 1, 1};
    EXPECT_FALSE(v_uint1l.is_zero());
    v_uint1l = Vector3ul();
    EXPECT_TRUE(v_uint1l.is_zero());
    v_uint1l[0] = 1;
    EXPECT_FALSE(v_uint1l.is_zero());
    const uint32_t testul1 = v_uint1l[0];
    EXPECT_TRUE(1 == testul1);
    v_uint1l.zero();
    EXPECT_TRUE(v_uint1l.is_zero());

    Vector3f v_float1(1.0f, 1.0f, 1.0f);
    Vector3f v_float2(1.0f, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(2.0f, v_float1 * v_float2);
    EXPECT_TRUE(Vector3f(-1.0f, 1.0f, 0.0f) == v_float1 % v_float2);
    v_float1 *= 2.0f;
    EXPECT_TRUE(Vector3f(2.0f, 2.0f, 2.0f) == v_float1);
    v_float1 /= 2.0f;
    EXPECT_TRUE(Vector3f(1.0f, 1.0f, 1.0f) == v_float1);
    v_float1 -= v_float2;
    EXPECT_TRUE(Vector3f(0.0f, 0.0f, 1.0f) == v_float1);
    v_float1 += v_float2;
    EXPECT_TRUE(Vector3f(1.0f, 1.0f, 1.0f) == v_float1);
    EXPECT_TRUE(Vector3f(nanf("0x4152"), 1.0f, 1.0f).is_nan());
    EXPECT_TRUE(Vector3f(1.0f / 0.0f, 1.0f, 1.0f).is_inf());
    EXPECT_TRUE(Vector3f(2.0f, 2.0f, 2.0f) / 2.0f == v_float1);
    EXPECT_TRUE(Vector3f(2.0f, 2.0f, 2.0f) == v_float1 * 2.0f);
    EXPECT_TRUE(Vector3f(2.0f, 2.0f, 2.0f) - v_float1 == v_float1);
    EXPECT_TRUE(Vector3f(2.0f, 2.0f, 2.0f) == v_float1 + v_float1);
    EXPECT_TRUE(Vector3f(-1.0f, -1.0f, -1.0f) == -v_float1);
    v_float1.zero();
    EXPECT_TRUE(v_float1.is_zero());
}

TEST(Vector3Test, OperatorDouble)
{
    Vector3d v_double0{1.0, 1.0,1.0};
    EXPECT_FALSE(v_double0.is_zero());
    v_double0 = Vector3d();
    EXPECT_TRUE(v_double0.is_zero());
    v_double0[1] = 1.0;
    EXPECT_FALSE(v_double0.is_zero());
    const double testf1 = v_double0[1];
    EXPECT_TRUE(is_equal(testf1, 1.0));
    v_double0.zero();
    EXPECT_TRUE(v_double0.is_zero());

    Vector3d v_double1(1.0, 1.0, 1.0);
    Vector3d v_double2(1.0, 1.0, 0.0);
    EXPECT_FLOAT_EQ(2.0, v_double1 * v_double2);
    EXPECT_TRUE(Vector3d(-1.0, 1.0, 0.0) == v_double1 % v_double2);
    v_double1 *= 2.0;
    EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0) == v_double1);
    v_double1 /= 2.0;
    EXPECT_TRUE(Vector3d(1.0, 1.0, 1.0) == v_double1);
    v_double1 -= v_double2;
    EXPECT_TRUE(Vector3d(0.0, 0.0, 1.0) == v_double1);
    v_double1 += v_double2;
    EXPECT_TRUE(Vector3d(1.0, 1.0, 1.0) == v_double1);
    EXPECT_TRUE(Vector3d(nan("0x4152"), 1.0, 1.0).is_nan());
    EXPECT_TRUE(Vector3d(1.0 / 0.0, 1.0, 1.0).is_inf());
    EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0) / 2.0 == v_double1);
    EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0) == v_double1 * 2.0);
    EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0) - v_double1 == v_double1);
    EXPECT_TRUE(Vector3d(2.0, 2.0, 2.0) == v_double1 + v_double1);
    EXPECT_TRUE(Vector3d(-1.0, -1.0, -1.0) == -v_double1);
    v_double1.zero();
    EXPECT_TRUE(v_double1.is_zero());
}

TEST(Vector3Test, IsEqual)
{
    Vector3f v_float1(1.0f, 1.0f, 1.0f);
    Vector3f v_float2(1.0f, 1.0f, 0.0f);

    EXPECT_FALSE(v_float1 == v_float2);
    EXPECT_TRUE(v_float1 == v_float1);
    EXPECT_TRUE(v_float1 != v_float2);
    EXPECT_FALSE(v_float1 != v_float1);

    Vector3d v_double1(1.0, 1.0, 1.0);
    Vector3d v_double2(1.0, 1.0, 0.0);

    EXPECT_FALSE(v_double1 == v_double2);
    EXPECT_TRUE(v_double1 == v_double1);
    EXPECT_TRUE(v_double1 != v_double2);
    EXPECT_FALSE(v_double1 != v_double1);
}
/*
TEST(Vector3Test, angle)
{
    EXPECT_FLOAT_EQ(M_PI/2, Vector3f(0.0f, 1.0f).angle(Vector3f(1.0f, 0.0f)));
    EXPECT_FLOAT_EQ(0.0f, Vector3f(0.5f, 0.5f).angle(Vector3f(0.5f, 0.5f)));
    EXPECT_FLOAT_EQ(M_PI, Vector3f(0.5f, -0.5f).angle(Vector3f(-0.5f, 0.5f)));

    EXPECT_FLOAT_EQ(0.0f, Vector3f(-0.0f, 0).angle(Vector3f(0.0f, 1.0f)));
}
*/
TEST(Vector3Test, length)
{
    EXPECT_FLOAT_EQ(12, Vector3f(2, 2, 2).length_squared());
    EXPECT_FLOAT_EQ(sqrtf(12), Vector3f(2, 2, 2).length());
    Vector3f v_float1(1.0f, 1.0f, 1.0f);
    EXPECT_TRUE(v_float1.limit_length_xy(1.0f));
    EXPECT_FALSE(Vector3f(-0.0f, -0.0f, -0.0f).limit_length_xy(1.0f));

    EXPECT_DOUBLE_EQ(12, Vector3d(2, 2, 2).length_squared());
    EXPECT_FLOAT_EQ(sqrt(12), Vector3d(2, 2, 2).length());
    Vector3d v_double1(1.0, 1.0, 1.0);
    EXPECT_TRUE(v_double1.limit_length_xy(1.0));
    EXPECT_FALSE(Vector3d(-0.0, -0.0, -0.0).limit_length_xy(1.0));
}

TEST(Vector3Test, normalized)
{
    Vector3f v_float1(3.0f, 3.0f, 3.0f);
    v_float1.normalize();
    EXPECT_EQ(Vector3f(3.0f, 3.0f, 3.0f).normalized(), v_float1);
    EXPECT_EQ(Vector3f(1 / sqrtf(3), 1 / sqrtf(3), 1 / sqrtf(3)), Vector3f(2, 2, 2).normalized());
    EXPECT_EQ(Vector3f(3, 3, 3).normalized(), Vector3f(5, 5, 5).normalized());
    EXPECT_EQ(Vector3f(-3, 3, 3).normalized(), Vector3f(-5, 5, 5).normalized());
    EXPECT_NE(Vector3f(-3, 3, 3).normalized(), Vector3f(5, 5, 5).normalized());
}
/*
TEST(Vector3Test, Project)
{
    Vector3f v_float1(1.0f, 1.0f, 1.0f);
    Vector3f v_float2(2.0f, 2.0f, 1.0f);
    v_float1.project(v_float2);
    EXPECT_EQ(Vector3f(1.0f, 1.0f, 1.0f).projected(v_float2), v_float1);
}

TEST(Vector3Test, reflect)
{
    Vector3f reflected1 = Vector3f(3, 3, 8);
    reflected1.reflect(Vector3f(0, 0, 1));
    EXPECT_EQ(reflected1, Vector3f(-3, -3, 8));

    // colinear vectors
    Vector3f reflected2 = Vector3f(3, 3, 3);
    reflected2.reflect(Vector3f(1, 1, 1));
    EXPECT_EQ(reflected2, Vector3f(3, 3, 3));

    // orthogonal vectors
    Vector3f reflected3 = Vector3f(3, 3, 3);
    reflected3.reflect(Vector3f(1, 1, -1));
    EXPECT_EQ(reflected3, Vector3f(-3, -3, -3));

    // rotation
    Vector3f base = Vector3f(2, 2, 1);
    base.rotate(radians(90));
    EXPECT_FLOAT_EQ(base.x, -1);
    EXPECT_FLOAT_EQ(base.y, 2);
    EXPECT_FLOAT_EQ(base.z, 2);
}

TEST(Vector3Test, Offset_bearing)
{
    Vector3f v_float1(1.0f, 0.0f);
    v_float1.offset_bearing(0.0f, 1.0f);
    EXPECT_EQ(Vector3f(2.0f, 0.0f), v_float1);
}

TEST(Vector3Test, Perpendicular)
{
    Vector3f v_float1(1.0f, 1.0f);
    EXPECT_EQ(Vector3f(0.0f, 2.0f), v_float1.perpendicular(v_float1, Vector3f(2.0f, 0.0f)));
    EXPECT_EQ(Vector3f(2.0f, 0.0f), v_float1.perpendicular(v_float1, Vector3f(0.0f, 2.0f)));
}

TEST(Vector3Test, closest_point)
{
    // closest_point is (p, v,w)

    // the silly case:
    EXPECT_EQ((Vector3f{0, 0}),
            (Vector3f::closest_point(Vector3f{0, 0}, Vector3f{0, 0}, Vector3f{0, 0})));
    // on line:
    EXPECT_EQ((Vector3f{0, 0}),
            (Vector3f::closest_point(Vector3f{0, 0}, Vector3f{0, 0}, Vector3f{1, 1})));
    EXPECT_EQ((Vector3f{5, 5}),
            (Vector3f::closest_point(Vector3f{5, 5}, Vector3f{0, 0}, Vector3f{5, 5})));
    // on line but not segment:
    EXPECT_EQ((Vector3f{5, 5}),
            (Vector3f::closest_point(Vector3f{6, 6}, Vector3f{0, 0}, Vector3f{5, 5})));

    EXPECT_EQ((Vector3f{0.5, 0.5}),
            (Vector3f::closest_point(Vector3f{1,0}, Vector3f{0, 0}, Vector3f{5, 5})));
    EXPECT_EQ((Vector3f{0, 1}),
            (Vector3f::closest_point(Vector3f{0,0}, Vector3f{-1, 1}, Vector3f{1, 1})));

    // to (0,w)
    // the silly case:
    EXPECT_EQ((Vector3f{0, 0}),
            (Vector3f::closest_point(Vector3f{0, 0}, Vector3f{0, 0})));
    // on line:
    EXPECT_EQ((Vector3f{0, 0}),
            (Vector3f::closest_point(Vector3f{0, 0}, Vector3f{1, 1})));
    EXPECT_EQ((Vector3f{5, 5}),
            (Vector3f::closest_point(Vector3f{5, 5}, Vector3f{5, 5})));
    // on line but not segment:
    EXPECT_EQ((Vector3f{5, 5}),
            (Vector3f::closest_point(Vector3f{6, 6}, Vector3f{5, 5})));

    EXPECT_EQ((Vector3f{0.5, 0.5}),
            (Vector3f::closest_point(Vector3f{1,0}, Vector3f{5, 5})));
    EXPECT_EQ((Vector3f{0, 0}),
            (Vector3f::closest_point(Vector3f{0,0}, Vector3f{1, 1})));
}

TEST(Vector3Test, closest_distance)
{
    EXPECT_FLOAT_EQ(1.0f, Vector3f::closest_distance_between_line_and_point_squared(Vector3f{0,0}, Vector3f{1, 0}, Vector3f{0, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector3f::closest_distance_between_line_and_point(Vector3f{0,0}, Vector3f{1, 0}, Vector3f{0, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector3f::closest_distance_between_lines_squared(Vector3f{0,0}, Vector3f{1, 0}, Vector3f{0, 1}, Vector3f{1, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector3f::closest_distance_between_radial_and_point_squared(Vector3f{0, 1}, Vector3f{1, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector3f::closest_distance_between_radial_and_point(Vector3f{0, 1}, Vector3f{1, 1}));
}

TEST(Vector3Test, segment_intersectionx)
{
    Vector3f intersection;
    EXPECT_EQ(Vector3f::segment_intersection(
            Vector3f{-1.0f, 0.0f}, // seg start
            Vector3f{1.0f, 0.0f}, // seg end
            Vector3f{0.0f, -1.0f}, // seg start
            Vector3f{0.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), true);
    EXPECT_EQ(intersection, Vector3f(0.0f, 0.0f));
    EXPECT_EQ(Vector3f::segment_intersection(
            Vector3f{1.0f, 0.0f}, // seg start
            Vector3f{2.0f, 0.0f}, // seg end
            Vector3f{0.0f, -1.0f}, // seg start
            Vector3f{0.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), false);
    EXPECT_EQ(Vector3f::segment_intersection(
            Vector3f{1.0f, 0.0f}, // seg start
            Vector3f{2.0f, 0.0f}, // seg end
            Vector3f{1.0f, 1.0f}, // seg start
            Vector3f{2.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), false);
}

TEST(Vector3Test, circle_segment_intersectionx)
{
    Vector3f intersection;
    EXPECT_EQ(Vector3f::circle_segment_intersection(
            Vector3f{0,0}, // seg start
            Vector3f{1,1}, // seg end
            Vector3f{0,0}, // circle center
            0.5,                 // circle radius
            intersection         // return value for intersection point
    ), true);
    EXPECT_EQ(intersection, Vector3f(sqrtf(0.5)/2,sqrtf(0.5)/2));

    EXPECT_EQ(Vector3f::circle_segment_intersection(
            Vector3f{std::numeric_limits<float>::quiet_NaN(),
                     std::numeric_limits<float>::quiet_NaN()}, // seg start
            Vector3f{1,1}, // seg end
            Vector3f{0,0}, // circle center
            0.5,                 // circle radius
            intersection         // return value for intersection point
    ), false);

}

TEST(Vector3Test, point_on_segmentx)
{
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{0.0f, 1.0f}, // point
            Vector3f{0.0f, 0.0f}, // seg start
            Vector3f{0.0f, 2.0f} // seg end
    ), true);
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{1.0f, 1.0f}, // point
            Vector3f{0.0f, 0.0f}, // seg start
            Vector3f{0.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{1.0f, 1.0f}, // point
            Vector3f{0.0f, 0.0f}, // seg start
            Vector3f{3.0f, 1.0f} // seg end
    ), false);
    printf("4\n");
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{1.0f, 0.0f}, // point
            Vector3f{2.0f, 1.0f}, // seg start
            Vector3f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{5.0f, 0.0f}, // point
            Vector3f{4.0f, 1.0f}, // seg start
            Vector3f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{3.0f, 0.0f}, // point
            Vector3f{3.0f, 1.0f}, // seg start
            Vector3f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector3f::point_on_segment(
            Vector3f{3.0f, 0.0f}, // point
            Vector3f{3.0f, 2.0f}, // seg start
            Vector3f{3.0f, 1.0f} // seg end
    ), false);

}
*/

// Project: v projected onto u = u * (v·u) / (u·u)
TEST(Vector3Test, Project)
{
    // Basic projection: (1,1,1) onto (2,2,1)
    // dot = 1*2+1*2+1*1 = 5, |v2|² = 4+4+1 = 9
    // result = (2,2,1) * 5/9
    Vector3f v1(1.0f, 1.0f, 1.0f);
    const Vector3f v2(2.0f, 2.0f, 1.0f);
    const Vector3f expected(10.0f/9.0f, 10.0f/9.0f, 5.0f/9.0f);
    EXPECT_NEAR(v1.projected(v2).x, expected.x, 1e-6f);
    EXPECT_NEAR(v1.projected(v2).y, expected.y, 1e-6f);
    EXPECT_NEAR(v1.projected(v2).z, expected.z, 1e-6f);

    // project() modifies in-place; result must match projected()
    v1.project(v2);
    EXPECT_NEAR(v1.x, expected.x, 1e-6f);
    EXPECT_NEAR(v1.y, expected.y, 1e-6f);
    EXPECT_NEAR(v1.z, expected.z, 1e-6f);

    // Orthogonal vectors: projection is zero
    const Vector3f ortho = Vector3f(1.0f, 0.0f, 0.0f).projected(Vector3f(0.0f, 1.0f, 0.0f));
    EXPECT_FLOAT_EQ(ortho.x, 0.0f);
    EXPECT_FLOAT_EQ(ortho.y, 0.0f);
    EXPECT_FLOAT_EQ(ortho.z, 0.0f);

    // Parallel: projection onto a parallel unit vector returns v unchanged
    const Vector3f v3(3.0f, 0.0f, 0.0f);
    const Vector3f proj = v3.projected(Vector3f(1.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(proj.x, 3.0f);
    EXPECT_FLOAT_EQ(proj.y, 0.0f);
    EXPECT_FLOAT_EQ(proj.z, 0.0f);
}

// Reflect: v' = 2*project(n) - v  (reflection about the axis n)
// - Components parallel to n are preserved
// - Components perpendicular to n are negated
TEST(Vector3Test, Reflect)
{
    // Reflect (3,3,8) about Z-axis: xy components negate, z preserved
    Vector3f r1(3.0f, 3.0f, 8.0f);
    r1.reflect(Vector3f(0.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(r1.x, -3.0f);
    EXPECT_FLOAT_EQ(r1.y, -3.0f);
    EXPECT_FLOAT_EQ(r1.z,  8.0f);

    // Collinear: reflect (3,3,3) about (1,1,1) — vector is along the axis, no change
    Vector3f r2(3.0f, 3.0f, 3.0f);
    r2.reflect(Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_NEAR(r2.x, 3.0f, 1e-5f);
    EXPECT_NEAR(r2.y, 3.0f, 1e-5f);
    EXPECT_NEAR(r2.z, 3.0f, 1e-5f);

    // Orthogonal: reflect (1,0,0) about Y-axis — negates x, preserves y, z
    // project (1,0,0) onto (0,1,0) = 0; reflect = 2*0 - (1,0,0) = (-1,0,0)
    Vector3f r3(1.0f, 0.0f, 0.0f);
    r3.reflect(Vector3f(0.0f, 1.0f, 0.0f));
    EXPECT_FLOAT_EQ(r3.x, -1.0f);
    EXPECT_FLOAT_EQ(r3.y,  0.0f);
    EXPECT_FLOAT_EQ(r3.z,  0.0f);

    // Mixed: reflect (1,2,3) about Z-axis
    // project(0,0,1): (0,0,3); reflect = (0,0,6) - (1,2,3) = (-1,-2,3)
    Vector3f r4(1.0f, 2.0f, 3.0f);
    r4.reflect(Vector3f(0.0f, 0.0f, 1.0f));
    EXPECT_FLOAT_EQ(r4.x, -1.0f);
    EXPECT_FLOAT_EQ(r4.y, -2.0f);
    EXPECT_FLOAT_EQ(r4.z,  3.0f);
}

// rotate_xy: rotate in the XY plane by angle_rad, leaving Z unchanged
TEST(Vector3Test, RotateXY)
{
    const float tol = 1e-6f;

    // (1,0,0) rotated 90° → (0,1,0)
    Vector3f v1(1.0f, 0.0f, 0.0f);
    v1.rotate_xy(M_PI/2);
    EXPECT_NEAR(v1.x,  0.0f, tol);
    EXPECT_NEAR(v1.y,  1.0f, tol);
    EXPECT_NEAR(v1.z,  0.0f, tol);

    // (0,1,0) rotated 90° → (-1,0,0)
    Vector3f v2(0.0f, 1.0f, 0.0f);
    v2.rotate_xy(M_PI/2);
    EXPECT_NEAR(v2.x, -1.0f, tol);
    EXPECT_NEAR(v2.y,  0.0f, tol);
    EXPECT_NEAR(v2.z,  0.0f, tol);

    // Z is preserved under XY rotation
    Vector3f v3(1.0f, 0.0f, 5.0f);
    v3.rotate_xy(M_PI/2);
    EXPECT_NEAR(v3.x,  0.0f, tol);
    EXPECT_NEAR(v3.y,  1.0f, tol);
    EXPECT_NEAR(v3.z,  5.0f, tol);

    // 180° rotation negates X and Y
    Vector3f v4(3.0f, 4.0f, 2.0f);
    v4.rotate_xy(M_PI);
    EXPECT_NEAR(v4.x, -3.0f, tol);
    EXPECT_NEAR(v4.y, -4.0f, tol);
    EXPECT_NEAR(v4.z,  2.0f, tol);

    // 0° rotation: identity
    Vector3f v5(1.0f, 2.0f, 3.0f);
    v5.rotate_xy(0.0f);
    EXPECT_NEAR(v5.x, 1.0f, tol);
    EXPECT_NEAR(v5.y, 2.0f, tol);
    EXPECT_NEAR(v5.z, 3.0f, tol);

    // 45° rotation: check exact position and length preservation
    // (3,4) rotated 45°: x'=(3-4)/√2, y'=(3+4)/√2
    Vector3f v6(3.0f, 4.0f, 7.0f);
    const float len_before = sqrtf(v6.x*v6.x + v6.y*v6.y);
    v6.rotate_xy(M_PI/4);
    EXPECT_NEAR(v6.x, (3.0f - 4.0f) / sqrtf(2.0f), 1e-5f);
    EXPECT_NEAR(v6.y, (3.0f + 4.0f) / sqrtf(2.0f), 1e-5f);
    EXPECT_NEAR(v6.z, 7.0f, tol);
    const float len_after = sqrtf(v6.x*v6.x + v6.y*v6.y);
    EXPECT_NEAR(len_after, len_before, 1e-5f);

    // Round-trip: rotating by θ then -θ returns to the original vector
    Vector3f v7(2.0f, 3.0f, 5.0f);
    const float angle = 1.234f;
    v7.rotate_xy(angle);
    v7.rotate_xy(-angle);
    EXPECT_NEAR(v7.x, 2.0f, 1e-5f);
    EXPECT_NEAR(v7.y, 3.0f, 1e-5f);
    EXPECT_NEAR(v7.z, 5.0f, tol);
}

// segment_plane_intersect: does a 3-D segment cross the given plane?
TEST(Vector3Test, SegmentPlaneIntersect)
{
    const Vector3f plane_normal(0.0f, 0.0f, 1.0f); // Z=0 plane
    const Vector3f plane_point(0.0f, 0.0f, 0.0f);

    // Segment straddles Z=0: start below, end above → intersects
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f),
        plane_normal, plane_point));

    // Segment entirely above Z=0 → does not intersect
    EXPECT_FALSE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, 1.0f), Vector3f(0.0f, 0.0f, 2.0f),
        plane_normal, plane_point));

    // Segment entirely below Z=0 → does not intersect
    EXPECT_FALSE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, -2.0f), Vector3f(0.0f, 0.0f, -1.0f),
        plane_normal, plane_point));

    // Segment lying entirely in the plane (D=0, N=0) → true
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(1.0f, 0.0f, 0.0f), Vector3f(0.0f, 1.0f, 0.0f),
        plane_normal, plane_point));

    // Segment parallel to plane but offset above (D=0, N≠0) → false
    EXPECT_FALSE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, 1.0f), Vector3f(1.0f, 0.0f, 1.0f),
        plane_normal, plane_point));

    // Segment start exactly on the plane → intersects
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 1.0f),
        plane_normal, plane_point));

    // Oblique segment, crossing at midpoint
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(1.0f, 1.0f, -1.0f), Vector3f(2.0f, 2.0f, 1.0f),
        plane_normal, plane_point));

    // Segment end exactly on the plane → intersects
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 0.0f, 1.0f), Vector3f(0.0f, 0.0f, 0.0f),
        plane_normal, plane_point));

    // Y-plane (normal=(0,1,0), passing through origin)
    const Vector3f ny(0.0f, 1.0f, 0.0f);
    const Vector3f py(0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, -1.0f, 0.0f), Vector3f(0.0f, 1.0f, 0.0f), ny, py));
    EXPECT_FALSE(Vector3f::segment_plane_intersect(
        Vector3f(0.0f, 1.0f, 0.0f), Vector3f(0.0f, 2.0f, 0.0f), ny, py));

    // Oblique plane: normal = (1,1,1)/√3, passing through origin.
    // Segment (1,0,0)→(-1,-1,-1): start is above (dot > 0), end is below (dot < 0).
    const Vector3f n_oblique(1.0f/sqrtf(3.0f), 1.0f/sqrtf(3.0f), 1.0f/sqrtf(3.0f));
    const Vector3f p_oblique(0.0f, 0.0f, 0.0f);
    EXPECT_TRUE(Vector3f::segment_plane_intersect(
        Vector3f(1.0f, 0.0f, 0.0f), Vector3f(-1.0f, -1.0f, -1.0f),
        n_oblique, p_oblique));
    // Segment entirely on the positive side
    EXPECT_FALSE(Vector3f::segment_plane_intersect(
        Vector3f(1.0f, 0.0f, 0.0f), Vector3f(0.0f, 1.0f, 0.0f),
        n_oblique, p_oblique));
}

// closest_distance_between_line_and_point: shortest distance from point p
// to the SEGMENT w1→w2 (clamps to endpoints, not infinite line).
TEST(Vector3Test, ClosestDistanceLine)
{
    // Segment along X-axis (0,0,0)→(10,0,0); point (0,1,0) is directly
    // above the start, within the segment's extent → foot = (0,0,0), dist = 1
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(0,1,0)),
        1.0f, 1e-6f);

    // Point (5,3,4) mid-segment: foot = (5,0,0), dist = sqrt(9+16) = 5
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(5,3,4)),
        5.0f, 1e-5f);

    // Point (3,0,0) lies ON the segment: distance = 0
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(3,0,0)),
        0.0f, 1e-5f);

    // Point beyond w2 endpoint: clamped to w2 = (10,0,0);
    // p = (15,0,0), dist = 5
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(15,0,0)),
        5.0f, 1e-5f);

    // 3-D case: segment (0,1,0)→(0,10,0), point (6,5,0)
    // Projection falls within segment, foot = (0,5,0), dist = 6
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(0,1,0), Vector3f(0,10,0), Vector3f(6,5,0)),
        6.0f, 1e-5f);

    // Degenerate (w1 == w2): returns distance to w1
    EXPECT_NEAR(
        Vector3f::closest_distance_between_line_and_point(
            Vector3f(3,4,0), Vector3f(3,4,0), Vector3f(0,0,0)),
        5.0f, 1e-5f);
}

// point_on_line_closest_to_other_point: foot of the perpendicular from p
// onto the SEGMENT w1→w2 (clamped — dot product constrained to [0,1])
TEST(Vector3Test, PointOnLineClosest)
{
    const float tol = 1e-5f;

    // Foot from (5,3,0) onto segment (0,0,0)→(10,0,0) → (5,0,0)
    Vector3f foot = Vector3f::point_on_line_closest_to_other_point(
        Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(5,3,0));
    EXPECT_NEAR(foot.x, 5.0f, tol);
    EXPECT_NEAR(foot.y, 0.0f, tol);
    EXPECT_NEAR(foot.z, 0.0f, tol);

    // Point beyond segment end → clamped to w2
    foot = Vector3f::point_on_line_closest_to_other_point(
        Vector3f(0,0,0), Vector3f(10,0,0), Vector3f(20,0,0));
    EXPECT_NEAR(foot.x, 10.0f, tol);
    EXPECT_NEAR(foot.y,  0.0f, tol);
    EXPECT_NEAR(foot.z,  0.0f, tol);

    // Point before segment start → clamped to w1
    foot = Vector3f::point_on_line_closest_to_other_point(
        Vector3f(5,0,0), Vector3f(10,0,0), Vector3f(0,0,0));
    EXPECT_NEAR(foot.x, 5.0f, tol);
    EXPECT_NEAR(foot.y, 0.0f, tol);
    EXPECT_NEAR(foot.z, 0.0f, tol);

    // Off-axis point near diagonal segment (0,0,0)→(0,10,10):
    // p=(3,5,5) — the projection along (0,1,1) is at parameter 0.5 → foot=(0,5,5)
    foot = Vector3f::point_on_line_closest_to_other_point(
        Vector3f(0,0,0), Vector3f(0,10,10), Vector3f(3,5,5));
    EXPECT_NEAR(foot.x, 0.0f, tol);
    EXPECT_NEAR(foot.y, 5.0f, tol);
    EXPECT_NEAR(foot.z, 5.0f, tol);

    // Degenerate: w1 == w2 → returns w1
    foot = Vector3f::point_on_line_closest_to_other_point(
        Vector3f(1,2,3), Vector3f(1,2,3), Vector3f(9,9,9));
    EXPECT_NEAR(foot.x, 1.0f, tol);
    EXPECT_NEAR(foot.y, 2.0f, tol);
    EXPECT_NEAR(foot.z, 3.0f, tol);
}

// NaN and Inf propagation through Vector3f arithmetic and operations.
TEST(Vector3Test, NaNAndInfPropagation)
{
    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    const float inf_val = std::numeric_limits<float>::infinity();

    // NaN propagates through addition and scalar multiply
    Vector3f v(1, 2, 3);
    v += Vector3f(nan_val, 0, 0);
    EXPECT_TRUE(v.is_nan());
    EXPECT_TRUE((Vector3f(1, 2, 3) * nan_val).is_nan());

    // Inf propagates through addition
    EXPECT_TRUE((Vector3f(1, 2, 3) + Vector3f(inf_val, 0, 0)).is_inf());

    // length_squared / length of a NaN vector is NaN
    const Vector3f vn(nan_val, 1, 1);
    EXPECT_TRUE(std::isnan(vn.length_squared()));
    EXPECT_TRUE(std::isnan(vn.length()));

    // normalize() on the zero vector produces NaN (division by zero)
    Vector3f zero;
    zero.normalize();
    EXPECT_TRUE(zero.is_nan());

    // project() onto the zero vector produces NaN (0/0 per IEEE 754)
    const Vector3f proj_onto_zero = Vector3f(1, 2, 3).projected(Vector3f(0, 0, 0));
    EXPECT_TRUE(proj_onto_zero.is_nan());
}

// Mathematical invariants for Vector3f operations.
TEST(Vector3Test, MathInvariants)
{
    const float tol = 1e-5f;

    // Double-reflection is identity: reflect(reflect(v, n), n) == v
    const Vector3f v(1.0f, 2.0f, 3.0f);
    const Vector3f n(0.0f, 0.0f, 1.0f);  // Z-axis
    Vector3f r = v;
    r.reflect(n);
    r.reflect(n);
    EXPECT_NEAR(r.x, v.x, tol);
    EXPECT_NEAR(r.y, v.y, tol);
    EXPECT_NEAR(r.z, v.z, tol);

    // Projection is idempotent: project(project(v, u), u) == project(v, u)
    const Vector3f u(1.0f, 1.0f, 0.0f);
    const Vector3f p1 = v.projected(u);
    const Vector3f p2 = p1.projected(u);
    EXPECT_NEAR(p1.x, p2.x, tol);
    EXPECT_NEAR(p1.y, p2.y, tol);
    EXPECT_NEAR(p1.z, p2.z, tol);

    // Orthogonal decomposition: v = proj(v,u) + residual, residual ⊥ u
    const Vector3f residual = v - p1;
    EXPECT_NEAR(residual * u, 0.0f, tol);  // dot product is zero

    // Rotation preserves XY length (checked separately for a non-trivial angle)
    Vector3f rv(3.0f, 4.0f, 2.0f);
    const float xy_len_before = sqrtf(rv.x*rv.x + rv.y*rv.y);
    rv.rotate_xy(0.7f);
    const float xy_len_after = sqrtf(rv.x*rv.x + rv.y*rv.y);
    EXPECT_NEAR(xy_len_after, xy_len_before, tol);
}

AP_GTEST_MAIN()
