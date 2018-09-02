#include <AP_gtest.h>

#include <AP_Math/vector2.h>
#include <AP_Math/vector3.h>

#define EXPECT_VECTOR2F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
    } while (false);

#define EXPECT_VECTOR3F_EQ(v1, v2)              \
    do {                                        \
        EXPECT_FLOAT_EQ(v1[0], v2[0]);          \
        EXPECT_FLOAT_EQ(v1[1], v2[1]);          \
        EXPECT_FLOAT_EQ(v1[2], v2[2]);          \
    } while (false);


#define PERP_TEST_2D(px,py, vx,vy, ex,ey)               \
    do {                                                \
        Vector2f p(px,py);                              \
        Vector2f v(vx,vy);                              \
        Vector2f expected(ex,ey);                       \
        Vector2f result;                                \
        result = Vector2f::perpendicular(p, v);         \
        EXPECT_VECTOR2F_EQ(expected, result);           \
    } while (false)

#define PERP_TEST_3D(px,py,pz, vx,vy,vz, ex,ey,ez)      \
    do {                                                \
        Vector3f p(px,py,pz);                           \
        Vector3f v(vx,vy,vz);                           \
        Vector3f expected(ex,ey,ez);                    \
        Vector3f result;                                \
        result = Vector3f::perpendicular(p, v);         \
        EXPECT_VECTOR3F_EQ(expected, result);           \
    } while (false)

void foo() { } 
TEST(ThreatTests, Distance)
{

    PERP_TEST_2D( 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f);
    PERP_TEST_2D( 0.0f,0.0f, 1.0f,0.0f, 0.0f,-1.0f);
    PERP_TEST_2D( 2.0f,0.0f, 1.0f,0.0f, 0.0f,-1.0f);
    PERP_TEST_2D( 0.0f,2.0f, 1.0f,0.0f, 0.0f,1.0f);
    PERP_TEST_2D( 0.0f,2.0f, 1.0f,2.0f, -2.0f,1.0f);
    PERP_TEST_2D( 2.0f,0.0f, 1.0f,2.0f, 2.0f,-1.0f);

    // 2D cases for the 3D code:
    PERP_TEST_3D( 0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f);
    PERP_TEST_3D( 0.0f,0.0f,0.0f, 1.0f,0.0f,0.0f, 0.0f,0.0f,0.0f);
    PERP_TEST_3D( 2.0f,0.0f,0.0f, 1.0f,0.0f,0.0f, 0.0f,0.0f,0.0f);
    foo();
    PERP_TEST_3D( 0.0f,2.0f,0.0f, 1.0f,0.0f,0.0f, 0.0f,2.0f,0.0f);
    PERP_TEST_3D( 0.0f,2.0f,0.0f, 1.0f,2.0f,0.0f, -0.8f,0.4f,0.0f);
    PERP_TEST_3D( 2.0f,0.0f,0.0f, 1.0f,2.0f,0.0f, 1.6f,-0.8f,0.0f);


    // 3D-specific tests
    PERP_TEST_3D( 1.0f,1.0f,1.0f, 1.0f,0.0f,0.0f, 0.0f,1.0f,1.0f);
    PERP_TEST_3D( -1.0f,-1.0f,-1.0f, -1.0f,0.0f,0.0f, 0.0f,-1.0f,-1.0f);
    PERP_TEST_3D(1.0f,1.0f,0.0f, 1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f);

    PERP_TEST_3D(1.0f,1.0f,1.0f, 1.0f,-1.0f,0.0f, 1.0f,1.0f,1.0f);

    PERP_TEST_3D(1.0f,1.0f,1.0f, 1.0f,-1.0f,1.0f, 0.66666666f,1.33333333f,0.66666666f);
}

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
