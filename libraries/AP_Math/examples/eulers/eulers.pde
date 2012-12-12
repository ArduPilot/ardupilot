/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math euler code
//

#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>

#include <AP_HAL_AVR.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

static float rad_diff(float rad1, float rad2)
{
    float diff = rad1 - rad2;
    if (diff > PI) {
        diff -= 2*PI;
    }
    if (diff < -PI) {
        diff += 2*PI;
    }
    return fabs(diff);
}

static void check_result(float roll, float pitch, float yaw,
                         float roll2, float pitch2, float yaw2)
{
    if (isnan(roll2) ||
        isnan(pitch2) ||
        isnan(yaw2)) {
        hal.console->printf("NAN eulers roll=%f pitch=%f yaw=%f\n",
                      roll, pitch, yaw);
    }

    if (rad_diff(roll2,roll) > ToRad(179)) {
        // reverse all 3
        roll2 += fmod(roll2+PI, 2*PI);
        pitch2 += fmod(pitch2+PI, 2*PI);
        yaw2 += fmod(yaw2+PI, 2*PI);
    }

    if (rad_diff(roll2,roll) > 0.01 ||
        rad_diff(pitch2, pitch) > 0.01 ||
        rad_diff(yaw2, yaw) > 0.01) {
        if (pitch >= PI/2 ||
            pitch <= -PI/2 ||
            ToDeg(rad_diff(pitch, PI/2)) < 1 ||
            ToDeg(rad_diff(pitch, -PI/2)) < 1) {
            // we expect breakdown at these poles
            hal.console->printf_P(
                    PSTR("breakdown eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n"),
                          ToDeg(roll), ToDeg(roll2),
                          ToDeg(pitch), ToDeg(pitch2),
                          ToDeg(yaw), ToDeg(yaw2));
        } else {
            hal.console->printf_P(
                    PSTR("incorrect eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n"),
                          ToDeg(roll), ToDeg(roll2),
                          ToDeg(pitch), ToDeg(pitch2),
                          ToDeg(yaw), ToDeg(yaw2));
        }
    }
}

static void test_euler(float roll, float pitch, float yaw)
{
    Matrix3f m;
    float roll2, pitch2, yaw2;

    m.from_euler(roll, pitch, yaw);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
}

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static const float angles[] = { 0, PI/8, PI/4, PI/2, PI,
                                -PI/8, -PI/4, -PI/2, -PI};

void test_matrix_eulers(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    hal.console->println("rotation matrix unit tests\n");

    for (i=0; i<N; i++)
        for (j=0; j<N; j++)
            for (k=0; k<N; k++)
                test_euler(angles[i], angles[j], angles[k]);

    hal.console->println("tests done\n");
}

static void test_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    float roll2, pitch2, yaw2;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(&roll2, &pitch2, &yaw2);
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
}

void test_quaternion_eulers(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    hal.console->println("quaternion unit tests\n");

    test_quaternion(PI/4, 0, 0);
    test_quaternion(0, PI/4, 0);
    test_quaternion(0, 0, PI/4);
    test_quaternion(-PI/4, 0, 0);
    test_quaternion(0, -PI/4, 0);
    test_quaternion(0, 0, -PI/4);
    test_quaternion(-PI/4, 1, 1);
    test_quaternion(1, -PI/4, 1);
    test_quaternion(1, 1, -PI/4);

    test_quaternion(ToRad(89), 0, 0.1);
    test_quaternion(0, ToRad(89), 0.1);
    test_quaternion(0.1, 0, ToRad(89));

    test_quaternion(ToRad(91), 0, 0.1);
    test_quaternion(0, ToRad(91), 0.1);
    test_quaternion(0.1, 0, ToRad(91));

    for (i=0; i<N; i++)
        for (j=0; j<N; j++)
            for (k=0; k<N; k++)
                test_quaternion(angles[i], angles[j], angles[k]);

    hal.console->println("tests done\n");
}


static void test_conversion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m, m2;

    float roll2, pitch2, yaw2;
    float roll3, pitch3, yaw3;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(&roll2, &pitch2, &yaw2);
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);

    q.rotation_matrix(m);
    m.to_euler(&roll2, &pitch2, &yaw2);

    m2.from_euler(roll, pitch, yaw);
    m2.to_euler(&roll3, &pitch3, &yaw3);
    if (m.is_nan()) {
        hal.console->printf("NAN matrix roll=%f pitch=%f yaw=%f\n",
                      roll, pitch, yaw);
    }

    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
    check_result(roll, pitch, yaw, roll3, pitch3, yaw3);
}

void test_conversions(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    hal.console->println("matrix/quaternion tests\n");

    test_conversion(1, 1.1, 1.2);
    test_conversion(1, -1.1, 1.2);
    test_conversion(1, -1.1, -1.2);
    test_conversion(-1, 1.1, -1.2);
    test_conversion(-1, 1.1, 1.2);

    for (i=0; i<N; i++)
        for (j=0; j<N; j++)
            for (k=0; k<N; k++)
                test_conversion(angles[i], angles[j], angles[k]);

    hal.console->println("tests done\n");
}

void test_frame_transforms(void)
{
    Vector3f v, v2;
    Quaternion q;
    Matrix3f m;

    hal.console->println("frame transform tests\n");

    q.from_euler(ToRad(90), 0, 0);
    v2 = v = Vector3f(0, 0, 1);
    q.earth_to_body(v2);
    hal.console->printf("%f %f %f\n", v2.x, v2.y, v2.z);
}

// generate a random float between -1 and 1
static float rand_num(void)
{
    float ret = ((unsigned)random()) % 2000000;
    return (ret - 1.0e6) / 1.0e6;
}

void test_matrix_rotate(void)
{
    Matrix3f m1, m2, diff;
    Vector3f r;

    m1.identity();
    m2.identity();
    r.x = rand_num();
    r.y = rand_num();
    r.z = rand_num();

    for (uint16_t i = 0; i<1000; i++) {
        // old method
        Matrix3f temp_matrix;
        temp_matrix.a.x = 0;
        temp_matrix.a.y = -r.z;
        temp_matrix.a.z =  r.y;
        temp_matrix.b.x =  r.z;
        temp_matrix.b.y = 0;
        temp_matrix.b.z = -r.x;
        temp_matrix.c.x = -r.y;
        temp_matrix.c.y =  r.x;
        temp_matrix.c.z = 0;
        temp_matrix = m1 * temp_matrix;
        m1 += temp_matrix;

        // new method
        m2.rotate(r);

        // check they behave in the same way
        diff = m1 - m2;
        float err = diff.a.length() + diff.b.length() + diff.c.length();

        if (err > 0) {
            hal.console->printf("ERROR: i=%u err=%f\n", (unsigned)i, err);
        }
    }
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->println("euler unit tests\n");

    test_conversion(0, PI, 0);

    test_frame_transforms();
    test_conversions();
    test_quaternion_eulers();
    test_matrix_eulers();
    test_matrix_rotate();
}

void loop(void){}

AP_HAL_MAIN();
