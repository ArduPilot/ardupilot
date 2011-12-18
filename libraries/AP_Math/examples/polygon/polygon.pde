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
};

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static float point_distance(Vector2l &p1, Vector2l &p2)
{
	float rads 			= (fabs(p1.x)*1.0e-7) * 0.0174532925;
	float lng_scale		= cos(rads);

	float dlat 		= (float)(p1.x - p2.x);
	float dlong		= ((float)(p1.y - p2.y)) * lng_scale;
	return sqrt((dlat*dlat) + (dlong*dlong)) * .01113195;
}

/*
  test precision of the calculation 
 */
static void precision_test(void)
{
    Vector2l p1, p2, p;
    float r;
    int32_t dx, dy;
    float worst_precision = 0.0;
    const float delta = 0.5;
    const float base = 0.9;

    Serial.println("Precision test:");

    p1 = OBC_boundary[8];
    p2 = OBC_boundary[10];
    dx = p2.x - p1.x;
    dy = p2.y - p1.y;
    
    // first come from the left
    for (r=-base; r<0.0; r *= delta) {
        p.x = p1.x + r*dx;
        p.y = p1.y + r*dy;
        if (!Polygon_outside(p, OBC_boundary, ARRAY_LENGTH(OBC_boundary))) {
            float precision = point_distance(p, p1);
            if (precision > worst_precision) {
                worst_precision = precision;
            }
        }
    }

    // in the middle
    for (r=base; r>0.0; r *= delta) {
        p.x = p1.x + r*dx;
        p.y = p1.y + r*dy;
        if (Polygon_outside(p, OBC_boundary, ARRAY_LENGTH(OBC_boundary))) {
            float precision = point_distance(p, p1);
            if (precision > worst_precision) {
                worst_precision = precision;
            }
        }
    }

    // from the left on other side
    for (r=-base; r<0.0; r *= delta) {
        p.x = p2.x + r*dx;
        p.y = p2.y + r*dy;
        if (Polygon_outside(p, OBC_boundary, ARRAY_LENGTH(OBC_boundary))) {
            float precision = point_distance(p, p2);
            if (precision > worst_precision) {
                worst_precision = precision;
            }
        }
    }

    // from the right 
    for (r=base; r>0.0; r *= delta) {
        p.x = p2.x + r*dx;
        p.y = p2.y + r*dy;
        if (!Polygon_outside(p, OBC_boundary, ARRAY_LENGTH(OBC_boundary))) {
            float precision = point_distance(p, p2);
            if (precision > worst_precision) {
                worst_precision = precision;
            }
        }
    }

    Serial.printf_P(PSTR("worst precision: %f meters\n"), worst_precision);
}

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
        result = Polygon_outside(test_points[i].point, OBC_boundary, ARRAY_LENGTH(OBC_boundary));
        Serial.printf_P(PSTR("%10f,%10f  %s  %s\n"),
                        1.0e-7*test_points[i].point.x,
                        1.0e-7*test_points[i].point.y,
                        result?"OUTSIDE":"INSIDE ",
                        result == test_points[i].outside?"PASS":"FAIL");
        if (result != test_points[i].outside) {
            all_passed = false;
        }
    }
    Serial.println(all_passed?"TEST PASSED":"TEST FAILED");

    precision_test();

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
    Serial.println(all_passed?"ALL TESTS PASSED":"TEST FAILED");                  
}

void
loop(void)
{
}
