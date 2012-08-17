/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math polygon code
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>

FastSerialPort(Serial, 0);

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

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

/*
 *  polygon tests
 */
void setup(void)
{
    unsigned i, count;
    bool all_passed = true;
    uint32_t start_time;

    Serial.begin(115200);
    Serial.println("polygon unit tests\n");

    if (!Polygon_complete(OBC_boundary, ARRAY_LENGTH(OBC_boundary))) {
        Serial.println("OBC boundary is not complete!");
        all_passed = false;
    }

    if (Polygon_complete(OBC_boundary, ARRAY_LENGTH(OBC_boundary)-1)) {
        Serial.println("Polygon_complete test failed");
        all_passed = false;
    }

    for (i=0; i<ARRAY_LENGTH(test_points); i++) {
        bool result;
        result = Polygon_outside(test_points[i].point, OBC_boundary, ARRAY_LENGTH(OBC_boundary));
        Serial.printf_P(PSTR("%10f,%10f  %s  %s\n"),
                        1.0e-7*test_points[i].point.x,
                        1.0e-7*test_points[i].point.y,
                        result ? "OUTSIDE" : "INSIDE ",
                        result == test_points[i].outside ? "PASS" : "FAIL");
        if (result != test_points[i].outside) {
            all_passed = false;
        }
    }
    Serial.println(all_passed ? "TEST PASSED" : "TEST FAILED");

    Serial.println("Speed test:");
    start_time = micros();
    for (count=0; count<1000; count++) {
        for (i=0; i<ARRAY_LENGTH(test_points); i++) {
            bool result;
            result = Polygon_outside(test_points[i].point, OBC_boundary, ARRAY_LENGTH(OBC_boundary));
            if (result != test_points[i].outside) {
                all_passed = false;
            }
        }
    }
    Serial.printf("%u usec/call\n", (unsigned)((micros() - start_time)/(count*ARRAY_LENGTH(test_points))));
    Serial.println(all_passed ? "ALL TESTS PASSED" : "TEST FAILED");
}

void
loop(void)
{
}
