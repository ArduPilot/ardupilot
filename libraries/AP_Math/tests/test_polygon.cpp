#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

#include <AP_Math/polygon.h>

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
