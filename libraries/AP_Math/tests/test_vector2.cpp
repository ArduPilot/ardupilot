#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(Vector2Test, Operator)
{
    Vector2f v_float0{1.0f, 1.0f};
    EXPECT_FALSE(v_float0.is_zero());
    v_float0 = Vector2f();
    EXPECT_TRUE(v_float0.is_zero());
    v_float0[1] = 1.0f;
    EXPECT_FALSE(v_float0.is_zero());
    const float testf1 = v_float0[1];
    EXPECT_TRUE(is_equal(testf1, 1.0f));
    v_float0.zero();
    EXPECT_TRUE(v_float0.is_zero());

    Vector2i v_inti1{1, 1};
    EXPECT_FALSE(v_inti1.is_zero());
    v_inti1 = Vector2i();
    EXPECT_TRUE(v_inti1.is_zero());
    v_inti1[0] = 1;
    EXPECT_FALSE(v_inti1.is_zero());
    const int16_t testi1 = v_inti1[0];
    EXPECT_TRUE(1 == testi1);
    v_inti1.zero();
    EXPECT_TRUE(v_inti1.is_zero());

    Vector2ui v_uinti1{1u, 1u};
    EXPECT_FALSE(v_uinti1.is_zero());
    v_uinti1 = Vector2ui();
    EXPECT_TRUE(v_uinti1.is_zero());
    v_uinti1[0] = 1u;
    EXPECT_FALSE(v_uinti1.is_zero());
    const uint16_t testui1 = v_uinti1[0];
    EXPECT_TRUE(1u == testui1);
    v_uinti1.zero();
    EXPECT_TRUE(v_uinti1.is_zero());

    Vector2l v_intl1{1, 1};
    EXPECT_FALSE(v_intl1.is_zero());
    v_intl1 = Vector2l();
    EXPECT_TRUE(v_intl1.is_zero());
    v_intl1[0] = 1;
    EXPECT_FALSE(v_intl1.is_zero());
    const int32_t testl1 = v_intl1[0];
    EXPECT_TRUE(1 == testl1);
    v_intl1.zero();
    EXPECT_TRUE(v_intl1.is_zero());

    Vector2ul v_uint1l{1, 1};
    EXPECT_FALSE(v_uint1l.is_zero());
    v_uint1l = Vector2ul();
    EXPECT_TRUE(v_uint1l.is_zero());
    v_uint1l[0] = 1;
    EXPECT_FALSE(v_uint1l.is_zero());
    const uint32_t testul1 = v_uint1l[0];
    EXPECT_TRUE(1 == testul1);
    v_uint1l.zero();
    EXPECT_TRUE(v_uint1l.is_zero());

    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(1.0f, 0.0f);
    EXPECT_FLOAT_EQ(1.0f, v_float1 * v_float2);
    EXPECT_FLOAT_EQ(-1.0f, v_float1 % v_float2);
    v_float1 *= 2.0f;
    EXPECT_TRUE(Vector2f(2.0f, 2.0f) == v_float1);
    v_float1 /= 2.0f;
    EXPECT_TRUE(Vector2f(1.0f, 1.0f) == v_float1);
    v_float1 -= v_float2;
    EXPECT_TRUE(Vector2f(0.0f, 1.0f) == v_float1);
    v_float1 += v_float2;
    EXPECT_TRUE(Vector2f(1.0f, 1.0f) == v_float1);
    EXPECT_TRUE(Vector2f(nanf("0x4152"), 1.0f).is_nan());
    EXPECT_TRUE(Vector2f(1.0f / 0.0f, 1.0f).is_inf());
    EXPECT_TRUE(Vector2f(2.0f, 2.0f) / 2.0f == v_float1);
    EXPECT_TRUE(Vector2f(2.0f, 2.0f) == v_float1 * 2.0f);
    EXPECT_TRUE(Vector2f(2.0f, 2.0f) - v_float1 == v_float1);
    EXPECT_TRUE(Vector2f(2.0f, 2.0f) == v_float1 + v_float1);
    EXPECT_TRUE(Vector2f(-1.0f, -1.0f) == -v_float1);
    v_float1.zero();
    EXPECT_TRUE(v_float1.is_zero());
}

TEST(Vector2Test, IsEqual)
{
    Vector2l v_int1(1, 1);
    Vector2l v_int2(1, 0);
    Vector2<long> v_long1(1, 1);
    Vector2<long> v_long2(1, 0);
    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(1.0f, 0.0f);

    EXPECT_FALSE(v_int1 == v_int2);
    EXPECT_TRUE(v_int1 == v_int1);
    EXPECT_TRUE(v_int1 != v_int2);
    EXPECT_FALSE(v_int1 != v_int1);
    EXPECT_FALSE(v_long1 == v_long2);
    EXPECT_TRUE(v_long1 == v_long1);
    EXPECT_TRUE(v_long1 != v_long2);
    EXPECT_FALSE(v_long1 != v_long1);
    EXPECT_FALSE(v_float1 == v_float2);
    EXPECT_TRUE(v_float1 == v_float1);
    EXPECT_TRUE(v_float1 != v_float2);
    EXPECT_FALSE(v_float1 != v_float1);
}

TEST(Vector2Test, angle)
{
    EXPECT_FLOAT_EQ(M_PI/2, Vector2f(0, 1).angle());
    EXPECT_FLOAT_EQ(M_PI/4, Vector2f(1, 1).angle());
    EXPECT_TRUE(is_zero(Vector2d(1, 0).angle()));
    EXPECT_FLOAT_EQ(-M_PI*3/4, Vector2f(-1, -1).angle());
    EXPECT_FLOAT_EQ(-M_PI*3/4, Vector2f(-5, -5).angle());

    // test all cardinal and inter-cardinal points:
    EXPECT_FLOAT_EQ(M_PI*0/4, Vector2f(1, 0).angle());
    EXPECT_FLOAT_EQ(M_PI*1/4, Vector2f(1, 1).angle());
    EXPECT_FLOAT_EQ(M_PI*2/4, Vector2f(0, 1).angle());
    EXPECT_FLOAT_EQ(M_PI*3/4, Vector2f(-1, 1).angle());
    EXPECT_FLOAT_EQ(M_PI*4/4, Vector2f(-1, 0).angle());
    EXPECT_FLOAT_EQ(-M_PI*3/4, Vector2f(-1, -1).angle());
    EXPECT_FLOAT_EQ(-M_PI*2/4, Vector2f(0, -1).angle());
    EXPECT_FLOAT_EQ(-M_PI*1/4, Vector2f(1, -1).angle());

    EXPECT_FLOAT_EQ(M_PI/2, Vector2f(0.0f, 1.0f).angle(Vector2f(1.0f, 0.0f)));
    EXPECT_FLOAT_EQ(0.0f, Vector2f(0.5f, 0.5f).angle(Vector2f(0.5f, 0.5f)));
    EXPECT_FLOAT_EQ(M_PI, Vector2f(0.5f, -0.5f).angle(Vector2f(-0.5f, 0.5f)));

    EXPECT_FLOAT_EQ(0.0f, Vector2f(-0.0f, 0).angle(Vector2f(0.0f, 1.0f)));
}

TEST(Vector2Test, length)
{
    EXPECT_FLOAT_EQ(25, Vector2f(3, 4).length_squared());
    Vector2f v_float1(1.0f, 1.0f);
    EXPECT_TRUE(v_float1.limit_length(1.0f));
    EXPECT_FALSE(Vector2f(-0.0f, 0.0f).limit_length(1.0f));
}

TEST(Vector2Test, normalized)
{
    Vector2f v_float1(3.0f, 3.0f);
    v_float1.normalize();
    EXPECT_EQ(Vector2f(3.0f, 3.0f).normalized(), v_float1);
    EXPECT_EQ(Vector2f(sqrtf(2)/2, sqrtf(2)/2), Vector2f(5, 5).normalized());
    EXPECT_EQ(Vector2f(3, 3).normalized(), Vector2f(5, 5).normalized());
    EXPECT_EQ(Vector2f(-3, 3).normalized(), Vector2f(-5, 5).normalized());
    EXPECT_NE(Vector2f(-3, 3).normalized(), Vector2f(5, 5).normalized());
}

TEST(Vector2Test, Project)
{
    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(2.0f, 1.0f);
    v_float1.project(v_float2);
    EXPECT_EQ(Vector2f(1.0f, 1.0f).projected(v_float2), v_float1);
}

TEST(Vector2Test, reflect)
{
    Vector2f reflected1 = Vector2f(3, 8);
    reflected1.reflect(Vector2f(0, 1));
    EXPECT_EQ(reflected1, Vector2f(-3, 8));

    // colinear vectors
    Vector2f reflected2 = Vector2f(3, 3);
    reflected2.reflect(Vector2f(1, 1));
    EXPECT_EQ(reflected2, Vector2f(3, 3));

    // orthogonal vectors
    Vector2f reflected3 = Vector2f(3, 3);
    reflected3.reflect(Vector2f(1, -1));
    EXPECT_EQ(reflected3, Vector2f(-3, -3));

    // rotation
    Vector2f base = Vector2f(2, 1);
    base.rotate(radians(90));
    EXPECT_FLOAT_EQ(base.x, -1);
    EXPECT_FLOAT_EQ(base.y, 2);
}

TEST(Vector2Test, Offset_bearing)
{
    Vector2f v_float1(1.0f, 0.0f);
    v_float1.offset_bearing(0.0f, 1.0f);
    EXPECT_EQ(Vector2f(2.0f, 0.0f), v_float1);
}

TEST(Vector2Test, Perpendicular)
{
    Vector2f v_float1(1.0f, 1.0f);
    EXPECT_EQ(Vector2f(0.0f, 2.0f), v_float1.perpendicular(v_float1, Vector2f(2.0f, 0.0f)));
    EXPECT_EQ(Vector2f(2.0f, 0.0f), v_float1.perpendicular(v_float1, Vector2f(0.0f, 2.0f)));
}

TEST(Vector2Test, closest_point)
{
    // closest_point is (p, v,w)

    // the silly case:
    EXPECT_EQ((Vector2f{0, 0}),
              (Vector2f::closest_point(Vector2f{0, 0}, Vector2f{0, 0}, Vector2f{0, 0})));
    // on line:
    EXPECT_EQ((Vector2f{0, 0}),
              (Vector2f::closest_point(Vector2f{0, 0}, Vector2f{0, 0}, Vector2f{1, 1})));
    EXPECT_EQ((Vector2f{5, 5}),
              (Vector2f::closest_point(Vector2f{5, 5}, Vector2f{0, 0}, Vector2f{5, 5})));
    // on line but not segment:
    EXPECT_EQ((Vector2f{5, 5}),
              (Vector2f::closest_point(Vector2f{6, 6}, Vector2f{0, 0}, Vector2f{5, 5})));

    EXPECT_EQ((Vector2f{0.5, 0.5}),
              (Vector2f::closest_point(Vector2f{1,0}, Vector2f{0, 0}, Vector2f{5, 5})));
    EXPECT_EQ((Vector2f{0, 1}),
              (Vector2f::closest_point(Vector2f{0,0}, Vector2f{-1, 1}, Vector2f{1, 1})));

    // to (0,w)
    // the silly case:
    EXPECT_EQ((Vector2f{0, 0}),
            (Vector2f::closest_point(Vector2f{0, 0}, Vector2f{0, 0})));
    // on line:
    EXPECT_EQ((Vector2f{0, 0}),
            (Vector2f::closest_point(Vector2f{0, 0}, Vector2f{1, 1})));
    EXPECT_EQ((Vector2f{5, 5}),
            (Vector2f::closest_point(Vector2f{5, 5}, Vector2f{5, 5})));
    // on line but not segment:
    EXPECT_EQ((Vector2f{5, 5}),
            (Vector2f::closest_point(Vector2f{6, 6}, Vector2f{5, 5})));

    EXPECT_EQ((Vector2f{0.5, 0.5}),
            (Vector2f::closest_point(Vector2f{1,0}, Vector2f{5, 5})));
    EXPECT_EQ((Vector2f{0, 0}),
            (Vector2f::closest_point(Vector2f{0,0}, Vector2f{1, 1})));
}

TEST(Vector2Test, closest_distance)
{
    EXPECT_FLOAT_EQ(1.0f, Vector2f::closest_distance_between_line_and_point_squared(Vector2f{0,0}, Vector2f{1, 0}, Vector2f{0, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector2f::closest_distance_between_line_and_point(Vector2f{0,0}, Vector2f{1, 0}, Vector2f{0, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector2f::closest_distance_between_lines_squared(Vector2f{0,0}, Vector2f{1, 0}, Vector2f{0, 1}, Vector2f{1, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector2f::closest_distance_between_radial_and_point_squared(Vector2f{0, 1}, Vector2f{1, 1}));
    EXPECT_FLOAT_EQ(1.0f, Vector2f::closest_distance_between_radial_and_point(Vector2f{0, 1}, Vector2f{1, 1}));
}

TEST(Vector2Test, segment_intersectionx)
{
    Vector2f intersection;
    EXPECT_EQ(Vector2f::segment_intersection(
            Vector2f{-1.0f, 0.0f}, // seg start
            Vector2f{1.0f, 0.0f}, // seg end
            Vector2f{0.0f, -1.0f}, // seg start
            Vector2f{0.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), true);
    EXPECT_EQ(intersection, Vector2f(0.0f, 0.0f));
    EXPECT_EQ(Vector2f::segment_intersection(
            Vector2f{1.0f, 0.0f}, // seg start
            Vector2f{2.0f, 0.0f}, // seg end
            Vector2f{0.0f, -1.0f}, // seg start
            Vector2f{0.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), false);
    EXPECT_EQ(Vector2f::segment_intersection(
            Vector2f{1.0f, 0.0f}, // seg start
            Vector2f{2.0f, 0.0f}, // seg end
            Vector2f{1.0f, 1.0f}, // seg start
            Vector2f{2.0f, 1.0f}, // seg end
            intersection         // return value for intersection point
    ), false);
}

TEST(Vector2Test, circle_segment_intersectionx)
{
    Vector2f intersection;
    EXPECT_EQ(Vector2f::circle_segment_intersection(
                  Vector2f{0,0}, // seg start
                  Vector2f{1,1}, // seg end
                  Vector2f{0,0}, // circle center
                  0.5,                 // circle radius
                  intersection         // return value for intersection point
                  ), true);
    EXPECT_EQ(intersection, Vector2f(sqrtf(0.5)/2,sqrtf(0.5)/2));

    EXPECT_EQ(Vector2f::circle_segment_intersection(
                  Vector2f{std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN()}, // seg start
                  Vector2f{1,1}, // seg end
                  Vector2f{0,0}, // circle center
                  0.5,                 // circle radius
                  intersection         // return value for intersection point
                  ), false);

}

TEST(Vector2Test, point_on_segmentx)
{
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{0.0f, 1.0f}, // point
            Vector2f{0.0f, 0.0f}, // seg start
            Vector2f{0.0f, 2.0f} // seg end
    ), true);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{1.0f, 1.0f}, // point
            Vector2f{0.0f, 0.0f}, // seg start
            Vector2f{0.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{1.0f, 1.0f}, // point
            Vector2f{0.0f, 0.0f}, // seg start
            Vector2f{3.0f, 1.0f} // seg end
    ), false);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{1.0f, 0.0f}, // point
            Vector2f{2.0f, 1.0f}, // seg start
            Vector2f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{5.0f, 0.0f}, // point
            Vector2f{4.0f, 1.0f}, // seg start
            Vector2f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{3.0f, 0.0f}, // point
            Vector2f{3.0f, 1.0f}, // seg start
            Vector2f{3.0f, 2.0f} // seg end
    ), false);
    EXPECT_EQ(Vector2f::point_on_segment(
            Vector2f{3.0f, 0.0f}, // point
            Vector2f{3.0f, 2.0f}, // seg start
            Vector2f{3.0f, 1.0f} // seg end
    ), false);

}

// Extended projection tests: correctness of projected() values and consistency
// between project() and projected()
TEST(Vector2Test, ProjectValues)
{
    // Basic: (3,4) projected onto (1,0) = (3,0)
    Vector2f v1(3.0f, 4.0f);
    v1.project(Vector2f(1.0f, 0.0f));
    EXPECT_FLOAT_EQ(v1.x, 3.0f);
    EXPECT_FLOAT_EQ(v1.y, 0.0f);

    // project() and projected() must be consistent
    const Vector2f v2(1.0f, 1.0f);
    const Vector2f onto(2.0f, 1.0f);
    // dot = 2+1 = 3, |onto|² = 5 → result = (2,1)*3/5 = (1.2, 0.6)
    const Vector2f p = v2.projected(onto);
    EXPECT_NEAR(p.x, 1.2f, 1e-6f);
    EXPECT_NEAR(p.y, 0.6f, 1e-6f);

    Vector2f v3 = v2;
    v3.project(onto);
    EXPECT_NEAR(v3.x, p.x, 1e-6f);
    EXPECT_NEAR(v3.y, p.y, 1e-6f);

    // Orthogonal: projection is zero
    const Vector2f ortho = Vector2f(1.0f, 0.0f).projected(Vector2f(0.0f, 1.0f));
    EXPECT_FLOAT_EQ(ortho.x, 0.0f);
    EXPECT_FLOAT_EQ(ortho.y, 0.0f);

    // Self-projection: result is the same vector (parallel)
    const Vector2f v4(3.0f, 4.0f);
    const Vector2f self_proj = v4.projected(v4);
    EXPECT_NEAR(self_proj.x, v4.x, 1e-5f);
    EXPECT_NEAR(self_proj.y, v4.y, 1e-5f);
}

// normalized() must return a unit vector (length == 1) and preserve direction
TEST(Vector2Test, NormalizedIsUnit)
{
    // Axis-aligned
    EXPECT_FLOAT_EQ(Vector2f(5.0f, 0.0f).normalized().length(), 1.0f);
    EXPECT_FLOAT_EQ(Vector2f(0.0f, 7.0f).normalized().length(), 1.0f);

    // Diagonal
    EXPECT_NEAR(Vector2f(3.0f, 4.0f).normalized().length(), 1.0f, 1e-6f);
    EXPECT_NEAR(Vector2f(-1.0f, -1.0f).normalized().length(), 1.0f, 1e-6f);

    // Direction is preserved: cross product == 0 and dot product > 0
    const Vector2f v(3.0f, 4.0f);
    const Vector2f n = v.normalized();
    // 2-D cross: v.x*n.y - v.y*n.x == 0 (parallel)
    EXPECT_NEAR(v.x * n.y - v.y * n.x, 0.0f, 1e-5f);
    // dot product > 0 (same half-plane, not opposite direction)
    EXPECT_GT(v * n, 0.0f);

    // normalize() in-place matches normalized()
    Vector2f v2(3.0f, 4.0f);
    v2.normalize();
    EXPECT_NEAR(v2.x, n.x, 1e-6f);
    EXPECT_NEAR(v2.y, n.y, 1e-6f);

    // Unit vector normalizes to itself
    const Vector2f unit(1.0f, 0.0f);
    EXPECT_EQ(unit.normalized(), unit);
}

// NaN and Inf propagation through Vector2f arithmetic and operations.
TEST(Vector2Test, NaNAndInfPropagation)
{
    const float nan_val = std::numeric_limits<float>::quiet_NaN();
    const float inf_val = std::numeric_limits<float>::infinity();

    // NaN propagates through addition and scalar multiply
    Vector2f v(1.0f, 2.0f);
    v += Vector2f(nan_val, 0.0f);
    EXPECT_TRUE(v.is_nan());
    EXPECT_TRUE((Vector2f(1.0f, 1.0f) * nan_val).is_nan());

    // Inf propagates through addition
    EXPECT_TRUE((Vector2f(1.0f, 2.0f) + Vector2f(inf_val, 0.0f)).is_inf());

    // normalize() on the zero vector produces NaN (division by zero)
    Vector2f zero;
    zero.normalize();
    EXPECT_TRUE(zero.is_nan());

    // project() onto the zero vector produces NaN (0/0 per IEEE 754)
    Vector2f v_proj(1.0f, 2.0f);
    v_proj.project(Vector2f(0.0f, 0.0f));
    EXPECT_TRUE(v_proj.is_nan());
}

// segment_intersection: additional boundary / degenerate cases
TEST(Vector2Test, SegmentIntersectionEdgeCases)
{
    Vector2f intersection;

    // T-intersection: one segment endpoint lies on the other segment
    EXPECT_TRUE(Vector2f::segment_intersection(
        Vector2f{0.0f, 0.0f}, Vector2f{2.0f, 0.0f},
        Vector2f{1.0f, -1.0f}, Vector2f{1.0f, 0.0f},
        intersection));
    EXPECT_NEAR(intersection.x, 1.0f, 1e-5f);
    EXPECT_NEAR(intersection.y, 0.0f, 1e-5f);

    // Collinear non-overlapping: no intersection
    EXPECT_FALSE(Vector2f::segment_intersection(
        Vector2f{0.0f, 0.0f}, Vector2f{1.0f, 0.0f},
        Vector2f{2.0f, 0.0f}, Vector2f{3.0f, 0.0f},
        intersection));

    // Shared endpoint: segments share exactly one endpoint
    EXPECT_TRUE(Vector2f::segment_intersection(
        Vector2f{0.0f, 0.0f}, Vector2f{1.0f, 0.0f},
        Vector2f{1.0f, 0.0f}, Vector2f{2.0f, 1.0f},
        intersection));
    EXPECT_NEAR(intersection.x, 1.0f, 1e-5f);
    EXPECT_NEAR(intersection.y, 0.0f, 1e-5f);
}

// closest_distance: additional cases not covered by the basic test
TEST(Vector2Test, ClosestDistanceEdgeCases)
{
    // Point on the segment: distance == 0 (use long-enough segment)
    EXPECT_NEAR(
        Vector2f::closest_distance_between_line_and_point(
            Vector2f{0,0}, Vector2f{10,0}, Vector2f{5,0}),
        0.0f, 1e-6f);

    // Diagonal segment: distance from off-axis point
    // Segment (0,0)→(1,1), point (1,0); foot = (0.5,0.5); distance = sqrt(0.5)
    EXPECT_NEAR(
        Vector2f::closest_distance_between_line_and_point(
            Vector2f{0,0}, Vector2f{1,1}, Vector2f{1,0}),
        sqrtf(0.5f), 1e-5f);

    // Point beyond segment end: clamped to w2=(10,0); distance = 5
    EXPECT_NEAR(
        Vector2f::closest_distance_between_line_and_point(
            Vector2f{0,0}, Vector2f{10,0}, Vector2f{15,0}),
        5.0f, 1e-5f);

    // Point before segment start: clamped to w1=(5,0); distance = 3
    EXPECT_NEAR(
        Vector2f::closest_distance_between_line_and_point(
            Vector2f{5,0}, Vector2f{10,0}, Vector2f{2,0}),
        3.0f, 1e-5f);

    // Degenerate (w1 == w2): returns distance to w1
    EXPECT_NEAR(
        Vector2f::closest_distance_between_line_and_point(
            Vector2f{3,4}, Vector2f{3,4}, Vector2f{0,0}),
        5.0f, 1e-5f);
}

AP_GTEST_MAIN()
