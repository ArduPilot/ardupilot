#include <AP_gtest.h>
#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include <AP_Math/AP_Math.h>

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
AP_GTEST_MAIN()
