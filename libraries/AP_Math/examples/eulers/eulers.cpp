//
// Unit tests for the AP_Math euler code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

void setup();
void loop();
void test_matrix_rotate(void);
void test_frame_transforms(void);
void test_conversions(void);
void test_quaternion_eulers(void);
void test_matrix_eulers(void);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SHOW_POLES_BREAKDOWN 0

static float rad_diff(float rad1, float rad2)
{
    float diff = rad1 - rad2;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return fabsf(diff);
}

static void check_result(const char *msg,
                         float roll, float pitch, float yaw,
                         float roll2, float pitch2, float yaw2)
{
    if (isnan(roll2) ||
        isnan(pitch2) ||
        isnan(yaw2)) {
        hal.console->printf("%s NAN eulers roll=%f pitch=%f yaw=%f\n",
                            msg,
                            (double)roll,
                            (double)pitch,
                            (double)yaw);
    }

    if (rad_diff(roll2,roll) > ToRad(179)) {
        // reverse all 3
        roll2 += fmodf(roll2 + M_PI, 2 * M_PI);
        pitch2 += fmodf(pitch2 + M_PI, 2 * M_PI);
        yaw2 += fmodf(yaw2 + M_PI, 2 * M_PI);
    }

    if (rad_diff(roll2,roll) > 0.01f ||
        rad_diff(pitch2, pitch) > 0.01f ||
        rad_diff(yaw2, yaw) > 0.01f) {
        if (pitch >= M_PI/2 ||
            pitch <= -M_PI/2 ||
            ToDeg(rad_diff(pitch, M_PI/2)) < 1 ||
            ToDeg(rad_diff(pitch, -M_PI/2)) < 1) {
            // we expect breakdown at these poles
#if SHOW_POLES_BREAKDOWN
            hal.console->printf(
                "%s breakdown eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                msg,
                (double)ToDeg(roll), (double)ToDeg(roll2),
                (double)ToDeg(pitch), (double)ToDeg(pitch2),
                (double)ToDeg(yaw), (double)ToDeg(yaw2));
#endif
        } else {
            hal.console->printf(
                "%s incorrect eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                msg,
                (double)ToDeg(roll), (double)ToDeg(roll2),
                (double)ToDeg(pitch), (double)ToDeg(pitch2),
                (double)ToDeg(yaw), (double)ToDeg(yaw2));
        }
    }
}

static void test_euler(float roll, float pitch, float yaw)
{
    Matrix3f m;
    float roll2, pitch2, yaw2;

    m.from_euler(roll, pitch, yaw);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_euler", roll, pitch, yaw, roll2, pitch2, yaw2);
}

static const float angles[] = { 0, M_PI/8, M_PI/4, M_PI/2, M_PI,
                                -M_PI/8, -M_PI/4, -M_PI/2, -M_PI};

void test_matrix_eulers(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("rotation matrix unit tests\n\n");

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_euler(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}

static void test_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m;
    float roll2, pitch2, yaw2;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_quaternion1", roll, pitch, yaw, roll2, pitch2, yaw2);

    m.from_euler(roll, pitch, yaw);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_quaternion2", roll, pitch, yaw, roll2, pitch2, yaw2);

    m.from_euler(roll, pitch, yaw);
    q.from_rotation_matrix(m);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_quaternion3", roll, pitch, yaw, roll2, pitch2, yaw2);

    q.rotation_matrix(m);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_quaternion4", roll, pitch, yaw, roll2, pitch2, yaw2);
}

void test_quaternion_eulers(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("quaternion unit tests\n\n");

    test_quaternion(M_PI/4, 0, 0);
    test_quaternion(0, M_PI/4, 0);
    test_quaternion(0, 0, M_PI/4);
    test_quaternion(-M_PI/4, 0, 0);
    test_quaternion(0, -M_PI/4, 0);
    test_quaternion(0, 0, -M_PI/4);
    test_quaternion(-M_PI/4, 1, 1);
    test_quaternion(1, -M_PI/4, 1);
    test_quaternion(1, 1, -M_PI/4);

    test_quaternion(ToRad(89), 0, 0.1f);
    test_quaternion(0, ToRad(89), 0.1f);
    test_quaternion(0.1f, 0, ToRad(89));

    test_quaternion(ToRad(91), 0, 0.1f);
    test_quaternion(0, ToRad(91), 0.1f);
    test_quaternion(0.1f, 0, ToRad(91));

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_quaternion(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}


static void test_conversion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m, m2;

    float roll2, pitch2, yaw2;
    float roll3, pitch3, yaw3;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_conversion1", roll, pitch, yaw, roll2, pitch2, yaw2);

    q.rotation_matrix(m);
    m.to_euler(&roll2, &pitch2, &yaw2);

    m2.from_euler(roll, pitch, yaw);
    m2.to_euler(&roll3, &pitch3, &yaw3);
    if (m.is_nan()) {
        hal.console->printf("NAN matrix roll=%f pitch=%f yaw=%f\n",
                            (double)roll,
                            (double)pitch,
                            (double)yaw);
    }

    check_result("test_conversion2", roll, pitch, yaw, roll2, pitch2, yaw2);
    check_result("test_conversion3", roll, pitch, yaw, roll3, pitch3, yaw3);
}

void test_conversions(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("matrix/quaternion tests\n\n");

    test_conversion(1, 1.1f, 1.2f);
    test_conversion(1, -1.1f, 1.2f);
    test_conversion(1, -1.1f, -1.2f);
    test_conversion(-1, 1.1f, -1.2f);
    test_conversion(-1, 1.1f, 1.2f);

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_conversion(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}

void test_frame_transforms(void)
{
    Vector3f v, v2;
    Quaternion q;
    Matrix3f m;

    hal.console->printf("frame transform tests\n\n");

    q.from_euler(ToRad(45), ToRad(45), ToRad(45));
    q.normalize();
    m.from_euler(ToRad(45), ToRad(45), ToRad(45));

    v2 = v = Vector3f(0.0f, 0.0f, 1.0f);
    q.body_to_earth(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n\n", (double)v2.x, (double)v2.y, (double)v2.z);

    v2 = v = Vector3f(0.0f, 1.0f, 0.0f);
    q.body_to_earth(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n\n", (double)v2.x, (double)v2.y, (double)v2.z);

    v2 = v = Vector3f(1.0f, 0.0f, 0.0f);
    q.body_to_earth(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
}

// generate a random float between -1 and 1
static float rand_num(void)
{
    return ((2.0f * get_random16()) / 0xFFFF) - 1.0f;
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

    for (uint16_t i = 0; i < 1000; i++) {
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
            hal.console->printf("ERROR: i=%u err=%f\n", (unsigned)i, (double)err);
        }
    }
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->printf("euler unit tests\n\n");

    test_conversion(0, M_PI, 0);

    test_frame_transforms();
    test_conversions();
    test_quaternion_eulers();
    test_matrix_eulers();
    test_matrix_rotate();
}

void loop(void) {}

AP_HAL_MAIN();
