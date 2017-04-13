//
// Unit tests for the AP_Math polygon code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
} test_points[] = {
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

/*
 *  polygon tests
 */
void setup(void)
{
    uint32_t count;
    bool all_passed = true;
    uint32_t start_time;

    hal.console->printf("polygon unit tests\n\n");

    if (!Polygon_complete(OBC_boundary, ARRAY_SIZE(OBC_boundary))) {
        hal.console->printf("OBC boundary is not complete!\n");
        all_passed = false;
    }

    if (Polygon_complete(OBC_boundary, ARRAY_SIZE(OBC_boundary)-1)) {
        hal.console->printf("Polygon_complete test failed\n");
        all_passed = false;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(test_points); i++) {
        bool result = Polygon_outside(test_points[i].point,
                OBC_boundary, ARRAY_SIZE(OBC_boundary));
        hal.console->printf("%10f,%10f  %s  %s\n",
                            (double)(1.0e-7f * test_points[i].point.x),
                            (double)(1.0e-7f * test_points[i].point.y),
                        result ? "OUTSIDE" : "INSIDE ",
                        result == test_points[i].outside ? "PASS" : "FAIL");
        if (result != test_points[i].outside) {
            all_passed = false;
        }
    }
    hal.console->printf("%s\n", all_passed ? "TEST PASSED" : "TEST FAILED");

    hal.console->printf("Speed test:\n");
    start_time = AP_HAL::micros();
    for (count = 0; count < 1000; count++) {
        for (uint32_t i = 0; i < ARRAY_SIZE(test_points); i++) {
            bool result = Polygon_outside(test_points[i].point,
                    OBC_boundary, ARRAY_SIZE(OBC_boundary));
            if (result != test_points[i].outside) {
                all_passed = false;
            }
        }
    }
    hal.console->printf("%u usec/call\n", (unsigned)((AP_HAL::micros()
                    - start_time)/(count * ARRAY_SIZE(test_points))));
    hal.console->printf("%s\n", all_passed ? "ALL TESTS PASSED" : "TEST FAILED");
}

void loop(void){}

AP_HAL_MAIN();
