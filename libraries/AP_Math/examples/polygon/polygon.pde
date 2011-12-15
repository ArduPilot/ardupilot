/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Meta_class and AP_Var classes.
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>

FastSerialPort(Serial, 0);

/*
  this is the boundary of the 2010 outback challenge
  Note that the last point must be the same as the first for the
  Polygon_outside() algorithm
 */
static const Vector2f OBC_boundary[] = {
	Vector2f(-26.569564000, 151.837373000),
	Vector2f(-26.569956000, 151.839405000),
	Vector2f(-26.576823000, 151.841142000),
	Vector2f(-26.577308000, 151.840344000),
	Vector2f(-26.581511000, 151.841950000),
	Vector2f(-26.578486000, 151.847469000),
	Vector2f(-26.599489000, 151.852886000),
	Vector2f(-26.609211000, 151.874742000),
	Vector2f(-26.645478000, 151.882053000),
	Vector2f(-26.643572000, 151.830350000),
	Vector2f(-26.587599000, 151.834405000),
	Vector2f(-26.569564000, 151.837373000)
};

static const struct {
	Vector2f point;
	bool outside;
} test_points[] = {
	{ Vector2f(-26.639887, 151.822281), true },
	{ Vector2f(-26.641870, 151.870926), false },
	{ Vector2f(-35.0,      149.0),      true },
	{ Vector2f(0, 0),                   true },
	{ Vector2f(-26.576815, 151.840825), false },
	{ Vector2f(-26.577406, 151.840586), true },
};

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

/*
  polygon tests
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
        result = Polygon_outside(&test_points[i].point, OBC_boundary, ARRAY_LENGTH(OBC_boundary));
        Serial.printf_P(PSTR("%10f,%10f  %s  %s\n"),
                        test_points[i].point.x, test_points[i].point.y,
                        result?"OUTSIDE":"INSIDE ",
                        result == test_points[i].outside?"PASS":"FAIL");
        if (result != test_points[i].outside) {
            all_passed = false;
        }
    }
    Serial.println(all_passed?"TEST PASSED":"TEST FAILED");

    Serial.println("Speed test:");
    start_time = micros();
    for (count=0; count<1000; count++) {
        for (i=0; i<ARRAY_LENGTH(test_points); i++) {
            bool result;
            result = Polygon_outside(&test_points[i].point, OBC_boundary, ARRAY_LENGTH(OBC_boundary));
            if (result != test_points[i].outside) {
                all_passed = false;
            }
        }
    }    
    Serial.printf("%u usec/call\n", (unsigned)((micros() - start_time)/(count*ARRAY_LENGTH(test_points))));
    Serial.println(all_passed?"ALL TESTS PASSED":"TEST FAILED");                  
}

void
loop(void)
{
}
