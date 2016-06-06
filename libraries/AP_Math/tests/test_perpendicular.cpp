#include <AP_gtest.h>

#include <AP_Math/vector2.h>

#define EXPECT_VECTOR2F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
    } while (false);

#define PERP_TEST(px,py, vx,vy, ex,ey)              \
    do {                                                \
        Vector2f p(px,py);                           \
        Vector2f v(vx,vy);                           \
        Vector2f expected(ex,ey);                       \
        Vector2f result;                                \
        result = Vector2f::perpendicular(p, v);   \
        EXPECT_VECTOR2F_EQ(result, expected);           \
    } while (false)

TEST(ThreatTests, Distance)
{

    PERP_TEST( 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f);
    PERP_TEST( 0.0f,0.0f, 1.0f,0.0f, 0.0f,-1.0f);
    PERP_TEST( 2.0f,0.0f, 1.0f,0.0f, 0.0f,-1.0f);
    PERP_TEST( 0.0f,2.0f, 1.0f,0.0f, 0.0f,1.0f);

    PERP_TEST( 0.0f,2.0f, 1.0f,2.0f, -2.0f,1.0f);
    PERP_TEST( 2.0f,0.0f, 1.0f,2.0f, 2.0f,-1.0f);
}

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
