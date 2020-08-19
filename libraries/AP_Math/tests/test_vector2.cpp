#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>

TEST(Vector2Test, IsEqual)
{
    Vector2l v_int1(1, 1);
    Vector2l v_int2(1, 0);
    Vector2f v_float1(1.0f, 1.0f);
    Vector2f v_float2(1.0f, 0.0f);

    EXPECT_FALSE(v_int1 == v_int2);
    EXPECT_TRUE(v_int1 == v_int1);
    EXPECT_FALSE(v_float1 == v_float2);
    EXPECT_TRUE(v_float1 == v_float1);
}

TEST(Vector2Test, angle)
{
    EXPECT_FLOAT_EQ(M_PI/2, Vector2f(0, 1).angle());
    EXPECT_FLOAT_EQ(M_PI/4, Vector2f(1, 1).angle());
    EXPECT_FLOAT_EQ(0.0f, Vector2f(1, 0).angle());
    EXPECT_FLOAT_EQ(M_PI*5/4, Vector2f(-1, -1).angle());
    EXPECT_FLOAT_EQ(M_PI*5/4, Vector2f(-5, -5).angle());
}

TEST(Vector2Test, length)
{
    EXPECT_FLOAT_EQ(25, Vector2f(3, 4).length_squared());
}

TEST(Vector2Test, normalized)
{
    EXPECT_EQ(Vector2f(sqrtf(2)/2, sqrtf(2)/2), Vector2f(5, 5).normalized());
    EXPECT_EQ(Vector2f(3, 3).normalized(), Vector2f(5, 5).normalized());
    EXPECT_EQ(Vector2f(-3, 3).normalized(), Vector2f(-5, 5).normalized());
    EXPECT_NE(Vector2f(-3, 3).normalized(), Vector2f(5, 5).normalized());
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
}

AP_GTEST_MAIN()
