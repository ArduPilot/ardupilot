#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

#include <AP_Math/AP_Math.h>

struct PB {
    Vector2f point;
    Vector2f boundary[3];
    bool outside;
};

static const PB points_boundaries[] = {
    { {0.1f,0.1f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, false },

     // test for winding order issues:
    { {0.9f,0.9f}, {{0.0f,0.0f}, {1.0f,0.0f}, {0.0f,1.0f}}, true },
    { {0.9f,0.9f}, {{0.0f,1.0f}, {0.0f,0.0f}, {1.0f,0.0f}}, true },
    { {0.9f,0.9f}, {{1.0f,0.0f}, {0.0f,1.0f}, {0.0f,0.0f}}, true },
    { {0.9f,0.9f}, {{0.0f,1.0f}, {1.0f,0.0f}, {0.0f,0.0f}}, true },

    { {0.1f,0.1f}, {{0.0f,0.0f}, {1.0f,0.0f}, {0.0f,1.0f}}, false },
    { {0.1f,0.1f}, {{0.0f,1.0f}, {0.0f,0.0f}, {1.0f,0.0f}}, false },
    { {0.1f,0.1f}, {{1.0f,0.0f}, {0.0f,1.0f}, {0.0f,0.0f}}, false },
    { {0.1f,0.1f}, {{0.0f,1.0f}, {1.0f,0.0f}, {0.0f,0.0f}}, false },

    { {0.99f-10.0f,0.99f-10.0f}, {{0.0f-10.0f,1.0f-10.0f}, {1.0f-10.0f,0.0f-10.0f}, {0.0f-10.0f,0.0f-10.0f}}, true },

    { {99.0f,99.0f}, {{0.0f,100.0f}, {100.0f,0.0f}, {0.0f,0.0f}}, true },

    { {0.1f,0.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, false },
    { {0.0f,0.99f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, false },
    { {0.99f,0.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, false },

    { {1.01f,0.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },
    { {0.0f,1.01f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },

    { {2.0f,2.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },
    { {-2.0f,-2.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },

    { {-0.05f,0.0f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },
    { {0.0f,-0.05f}, {{0.0f,0.0f}, {0.0f,1.0f}, {1.0f,0.0f}}, true },
};

TEST(Polygon, outside)
{
    // uint8_t count = 0;
    for (const struct PB &pb : points_boundaries) {
        // ::fprintf(stderr, "count=%u\n", count++);
        Vector2f v[4];
        memcpy(v, pb.boundary, sizeof(pb.boundary));
        v[3] = v[0]; // close it
        EXPECT_EQ(pb.outside, Polygon_outside(pb.point, v, 4));
    }
}

struct SquareBoundary {
    Vector2f point;
    Vector2f boundary[4];
    bool outside;
};
static const SquareBoundary square_boundaries[] = {
    { {1.0f,1.0f}, {{0.0f,0.0f}, {0.0f,10.0f}, {10.0, 10.0}, {10.0f,0.0f}}, false },
    { {9.0f,9.0f}, {{0.0f,0.0f}, {0.0f,10.0f}, {10.0, 10.0}, {10.0f,0.0f}}, false },
    { {1.0f,9.0f}, {{0.0f,0.0f}, {0.0f,10.0f}, {10.0, 10.0}, {10.0f,0.0f}}, false },
    { {9.0f,1.0f}, {{0.0f,0.0f}, {0.0f,10.0f}, {10.0, 10.0}, {10.0f,0.0f}}, false },

};

TEST(Polygon, square_boundaries)
{
    // uint8_t count = 0;
    for (const auto &pb : square_boundaries) {
        // ::fprintf(stderr, "count=%u\n", count++);
        Vector2f v[5];
        memcpy(v, pb.boundary, sizeof(pb.boundary));
        v[4] = v[0]; // close it
        EXPECT_EQ(pb.outside, Polygon_outside(pb.point, v, 5));
        EXPECT_EQ(Polygon_outside(pb.point, v, 4),
                  Polygon_outside(pb.point, v, 5));
    }
}

TEST(Polygon, circle_outside_triangle)
{
    const Vector2f triangle[] = {{0.0f,0.0f}, {1.0f,0.0f}, {0.0f,1.0f}};
    Vector2f triangle_closed[4];
    memcpy(triangle_closed, triangle, sizeof(triangle));
    triangle_closed[3] = triangle_closed[0];
    const float radius = 0.8f;
    for (uint16_t i=0; i<360; i++) {
        const float x = radius * sin(radians(i)) + 0.5f;
        const float y = radius * cos(radians(i)) + 0.5f;
        EXPECT_EQ(true, Polygon_outside(Vector2f{x,y}, triangle_closed, 4));
        EXPECT_EQ(Polygon_outside(Vector2f{x,y}, triangle_closed, 3),
                  Polygon_outside(Vector2f{x,y}, triangle_closed, 4));
    }
}

TEST(Polygon, circle_inside_triangle)
{
    const Vector2f triangle[] = {{0.0f,0.0f}, {1.0f,0.0f}, {0.0f,1.0f}};
    Vector2f triangle_closed[4];
    memcpy(triangle_closed, triangle, sizeof(triangle));
    triangle_closed[3] = triangle_closed[0];
    const float radius = 0.2f;
    for (uint16_t i=0; i<360; i++) {
        const float x = radius * sin(radians(i)) + 0.2f;
        const float y = radius * cos(radians(i)) + 0.2f;
        EXPECT_EQ(false, Polygon_outside(Vector2f{x,y}, triangle_closed, 4));
        EXPECT_EQ(Polygon_outside(Vector2f{x,y}, triangle_closed, 3),
                  Polygon_outside(Vector2f{x,y}, triangle_closed, 4));
    }
}

TEST(Polygon, complex)
{
    const Vector2f poly[] = {
        {0.0f,0.0f},
        {0.0f,10.0f},
        {5.0, 10.0f},
        {5.0f,5.0f},
        {3.0f,5.0f},
        {3.0f,6.0f},
        {4.0f,6.0f},
        {4.0f,9.0f},
        {4.0f,9.0f},
        {1.0f,9.0f},
        {1.0f,6.0f},
        {2.0f,6.0f},
        {2.0f,5.0f},
        {1.0f,5.0f},
        {1.0f,0.0f},
    };
    const Vector2f inside_points[] = {
        {0.1f, 0.1f},
        {4.5f, 9.5f},
        {0.5f, 9.5f},
    };
    const Vector2f outside_points[] = {
        {3.0f, 8.0f},
        {5.5f, 10.0f},
        {2.0f, 2.0f},
        {2.5f, 5.5f},
        {1.5f, 6.5f},
    };

    Vector2f closed_poly[sizeof(poly) + sizeof(Vector2f)];
    memcpy(closed_poly, poly, sizeof(poly));
    const uint16_t n = ARRAY_SIZE(closed_poly);
    closed_poly[n-1] = closed_poly[0];

    for (const auto &point : inside_points) {
        EXPECT_EQ(false, Polygon_outside(point, closed_poly, n));
        EXPECT_EQ(Polygon_outside(point, closed_poly, n-1),
                  Polygon_outside(point, closed_poly, n));
    }
    for (const auto &point : outside_points) {
        EXPECT_EQ(true, Polygon_outside(point, closed_poly, n));
        EXPECT_EQ(Polygon_outside(point, closed_poly, n-1),
                  Polygon_outside(point, closed_poly, n));
    }
}

TEST(Polygon, circle_outside_square)
{
    const Vector2f square[] = {{0.0f,0.0f}, {0.0f,10.0f}, {10.0, 10.0}, {10.0f,0.0f}};
    Vector2f square_closed[5];
    memcpy(square_closed, square, sizeof(square));
    square_closed[4] = square_closed[0];
    const float radius = 8.0f;
    for (uint16_t i=0; i<360; i++) {
        const float x = radius * sin(radians(i)) + 5.0f;
        const float y = radius * cos(radians(i)) + 5.0f;
        EXPECT_EQ(true, Polygon_outside(Vector2f{x,y}, square_closed, 4));
        EXPECT_EQ(Polygon_outside(Vector2f{x,y}, square_closed, 3),
                  Polygon_outside(Vector2f{x,y}, square_closed, 4));
    }
}

struct PB_long {
    Vector2l point;
    Vector2l boundary[3];
    bool outside;
};

static const PB_long points_boundaries_long[] = {
    { {1000000,1000000}, {{0,0}, {0,10000000}, {10000000,0}}, false },

     // test for winding order issues:
    { {9000000,9000000}, {{0,0}, {10000000,0}, {0,10000000}}, true },
    { {9000000,9000000}, {{0,10000000}, {0,0}, {10000000,0}}, true },
    { {9000000,9000000}, {{10000000,0}, {0,10000000}, {0,0}}, true },
    { {9000000,9000000}, {{0,10000000}, {10000000,0}, {0,0}}, true },

    { {1000000,1000000}, {{0,0}, {10000000,0}, {0,10000000}}, false },
    { {1000000,1000000}, {{0,10000000}, {0,0}, {10000000,0}}, false },
    { {1000000,1000000}, {{10000000,0}, {0,10000000}, {0,0}}, false },
    { {1000000,1000000}, {{0,10000000}, {10000000,0}, {0,0}}, false },

    { {9900000-10,9900000-10}, {{0-10,10000000-10}, {10000000-10,0-10}, {0-10,0-10}}, true },

    { {990000000,990000000}, {{0,100}, {100,0}, {0,0}}, true },

    { {1000000,0}, {{0,0}, {0,10000000}, {10000000,0}}, false },
    { {0,9900000}, {{0,0}, {0,10000000}, {10000000,0}}, false },
    { {9900000,0}, {{0,0}, {0,10000000}, {10000000,0}}, false },

    { {10100000,0}, {{0,0}, {0,10000000}, {10000000,0}}, true },
    { {0,10100000}, {{0,0}, {0,10000000}, {10000000,0}}, true },

    { {20000000,20000000}, {{0,0}, {0,10000000}, {10000000,0}}, true },
    { {-20000000,-20000000}, {{0,0}, {0,10000000}, {10000000,0}}, true },

    { {-500000,0}, {{0,0}, {0,10000000}, {10000000,0}}, true },
    { {0,-500000}, {{0,0}, {0,10000000}, {10000000,0}}, true },
};

TEST(Polygon, outside_long)
{
    // uint8_t count = 0;
    for (const struct PB_long &pb : points_boundaries_long) {
        // ::fprintf(stderr, "count=%u\n", count++);
        Vector2l v[4];
        memcpy(v, pb.boundary, sizeof(pb.boundary));
        v[3] = v[0]; // close it
        EXPECT_EQ(pb.outside, Polygon_outside(pb.point, v, 4));
    }
}

TEST(Polygon, outside_long_closed_equal_to_unclosed)
{
    // uint8_t count = 0;
    for (const struct PB_long &pb : points_boundaries_long) {
        // ::fprintf(stderr, "count=%u\n", count++);
        Vector2l v[4];
        memcpy(v, pb.boundary, sizeof(pb.boundary));
        v[3] = v[0]; // close it
        EXPECT_EQ(Polygon_outside(pb.point, v, 3),
                  Polygon_outside(pb.point, v, 4));
    }
}


#define TEST_POLYGON_POINTS(POLYGON, TEST_POINTS)                       \
    do {                                                                \
        for (uint32_t i = 0; i < ARRAY_SIZE(TEST_POINTS); i++) {        \
            EXPECT_EQ(TEST_POINTS[i].outside,                           \
                      Polygon_outside(TEST_POINTS[i].point,             \
                                      POLYGON, ARRAY_SIZE(POLYGON)));   \
        }                                                               \
    } while(0)

// this OBC polygon test stolen from the polygon Math example

/*
 *  this is the boundary of the 2010 outback challenge
 *  Note that the last point must be the same as the first for the
 *  Polygon_outside() algorithm
 */
static const Vector2l OBC_boundary[] = {
    Vector2l(-265695640, 1518373730),
    Vector2l(-265699560, 1518394050),
    Vector2l(-265768230, 1518411420),
    Vector2l(-265773080, 1518403440),
    Vector2l(-265815110, 1518419500),
    Vector2l(-265784860, 1518474690),
    Vector2l(-265994890, 1518528860),
    Vector2l(-266092110, 1518747420),
    Vector2l(-266454780, 1518820530),
    Vector2l(-266435720, 1518303500),
    Vector2l(-265875990, 1518344050),
    Vector2l(-265695640, 1518373730)
};

static const struct {
    Vector2l point;
    bool outside;
} OBC_test_points[] = {
    { Vector2l(-266398870, 1518220000), true },
    { Vector2l(-266418700, 1518709260), false },
    { Vector2l(-350000000, 1490000000), true },
    { Vector2l(0, 0),                   true },
    { Vector2l(-265768150, 1518408250), false },
    { Vector2l(-265774060, 1518405860), true },
    { Vector2l(-266435630, 1518303440), true },
    { Vector2l(-266435650, 1518313540), false },
    { Vector2l(-266435690, 1518303530), false },
    { Vector2l(-266435690, 1518303490), true },
    { Vector2l(-265875990, 1518344049), true },
    { Vector2l(-265875990, 1518344051), false },
    { Vector2l(-266454781, 1518820530), true },
    { Vector2l(-266454779, 1518820530), true },
    { Vector2l(-266092109, 1518747420), true },
    { Vector2l(-266092111, 1518747420), false },
    { Vector2l(-266092110, 1518747421), true },
    { Vector2l(-266092110, 1518747419), false },
    { Vector2l(-266092111, 1518747421), true },
    { Vector2l(-266092109, 1518747421), true },
    { Vector2l(-266092111, 1518747419), false },
};

TEST(Polygon, obc)
{
    TEST_POLYGON_POINTS(OBC_boundary, OBC_test_points);
}

static const Vector2f PROX_boundary[] = {
    Vector2f{938.315063f,388.662872f},
    Vector2f{545.622803f,1317.25f},
    Vector2f{-833.382812f,2011.96423f},
    Vector2f{-2011.96411f,833.382996f},
    Vector2f{-875.159241f,-362.502838f},
    Vector2f{-153.222916f,-369.912689f},
    Vector2f{153.222931f,-369.912689f},
    Vector2f{369.91272f,-153.222855f},

    // closing point so we can call Polygon_outside(...):
    Vector2f{938.315063f,388.662872f},
};


static const struct {
    Vector2f point;
    bool outside;
} PROX_test_points[] = {
    { Vector2f{0.0f,0.0f}, false },
};

TEST(Polygon, prox)
{
    TEST_POLYGON_POINTS(PROX_boundary, PROX_test_points);
}

static const Vector2f SIMPLE_boundary[] = {
    Vector2f{-1,2},
    Vector2f{1,2},
    Vector2f{1,-3},
    Vector2f{-1,-3},

    // closing point so we can call Polygon_outside(...):
    Vector2f{-1,2},
};

static const struct {
    Vector2f point;
    bool outside;
} SIMPLE_test_points[] = {
    { Vector2f{0.0f,0.0f}, false },
    { Vector2f{0.5f,1.5f}, false },
    { Vector2f{-0.5f,1.5f}, false },
    { Vector2f{-0.5f,-2.5}, false },
};

TEST(Polygon, simple)
{
    TEST_POLYGON_POINTS(SIMPLE_boundary, SIMPLE_test_points);
}

AP_GTEST_MAIN()


int hal = 0; // bizarrely, this fixes an undefined-symbol error but doesn't raise a type exception.  Yay.
