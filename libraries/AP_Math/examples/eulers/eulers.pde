/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math euler code
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>

FastSerialPort(Serial, 0);

#ifdef DESKTOP_BUILD
// all of this is needed to build with SITL
#include <DataFlash.h>
#include <APM_RC.h>
#include <GCS_MAVLink.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
Arduino_Mega_ISR_Registry isr_registry;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL     compass;
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
        Serial.printf("NAN eulers roll=%f pitch=%f yaw=%f\n",
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
        if (ToDeg(rad_diff(pitch, PI/2)) < 1 ||
            ToDeg(rad_diff(pitch, -PI/2)) < 1) {
            // we expect breakdown at these poles
            Serial.printf("breakdown eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                          ToDeg(roll), ToDeg(roll2), ToDeg(pitch), ToDeg(pitch2), ToDeg(yaw), ToDeg(yaw2));
        } else {
            Serial.printf("incorrect eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                          ToDeg(roll), ToDeg(roll2), ToDeg(pitch), ToDeg(pitch2), ToDeg(yaw), ToDeg(yaw2));
        }
    } else {
        Serial.printf("correct eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                      ToDeg(roll), ToDeg(roll2), ToDeg(pitch), ToDeg(pitch2), ToDeg(yaw), ToDeg(yaw2));
    }
}

static void test_euler(float roll, float pitch, float yaw)
{
    Matrix3f m;
    float roll2, pitch2, yaw2;

    rotation_matrix_from_euler(m, roll, pitch, yaw);
    calculate_euler_angles(m, &roll2, &pitch2, &yaw2);
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
}

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static const float angles[] = { 0, PI/8, PI/4, PI/2, PI,
                                -PI/8, -PI/4, -PI/2, -PI};

void test_matrix_eulers(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    Serial.println("rotation matrix unit tests\n");

    for (i=0; i<N; i++)
        for (j=0; j<N; j++)
            for (k=0; k<N; k++)
                test_euler(angles[i], angles[j], angles[k]);

    Serial.println("tests done\n");
}

static void test_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    float roll2, pitch2, yaw2;

    quaternion_from_euler(q, roll, pitch, yaw);
    euler_from_quaternion(q, &roll2, &pitch2, &yaw2);
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
}

void test_quaternion_eulers(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    Serial.println("quaternion unit tests\n");

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

    Serial.println("tests done\n");
}


static void test_conversion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m, m2;

    float roll2, pitch2, yaw2;

    quaternion_from_euler(q, roll, pitch, yaw);
    quaternion_to_rotation_matrix(q, m);
    rotation_matrix_from_euler(m2, roll, pitch, yaw);

    calculate_euler_angles(m, &roll2, &pitch2, &yaw2);
    if (m.is_nan()) {
        Serial.printf("NAN matrix roll=%f pitch=%f yaw=%f\n",
                      roll, pitch, yaw);
    }
    check_result(roll, pitch, yaw, roll2, pitch2, yaw2);
}

void test_conversions(void)
{
    uint8_t i, j, k;
    uint8_t N = ARRAY_LENGTH(angles);

    Serial.println("matrix/quaternion tests\n");

    test_conversion(1, 1.1, 1.2);
    test_conversion(1, -1.1, 1.2);
    test_conversion(1, -1.1, -1.2);
    test_conversion(-1, 1.1, -1.2);
    test_conversion(-1, 1.1, 1.2);

    for (i=0; i<N; i++)
        for (j=0; j<N; j++)
            for (k=0; k<N; k++)
                test_conversion(angles[i], angles[j], angles[k]);

    Serial.println("tests done\n");
}

void test_frame_transforms(void)
{
    Vector3f v, v2;
    Quaternion q;
    Matrix3f m;

    Serial.println("frame transform tests\n");

    quaternion_from_euler(q, ToRad(90), 0, 0);
    v2 = v = Vector3f(0, 0, 1);
    quaternion_earth_to_body(q, v2);
    printf("%f %f %f\n", v2.x, v2.y, v2.z);
}

/*
  euler angle tests
 */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("euler unit tests\n");
    test_frame_transforms();
    test_conversions();
    test_quaternion_eulers();
    test_matrix_eulers();
}

void
loop(void)
{
}
