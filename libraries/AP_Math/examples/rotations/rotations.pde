/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math rotations code
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


// standard rotation matrices (these are the originals from the old code)
#define MATRIX_ROTATION_NONE               Matrix3f(1, 0, 0, 0, 1, 0, 0 ,0, 1)
#define MATRIX_ROTATION_YAW_45             Matrix3f(0.70710678, -0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_90             Matrix3f(0, -1, 0, 1, 0, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_135            Matrix3f(-0.70710678, -0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_180            Matrix3f(-1, 0, 0, 0, -1, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_225            Matrix3f(-0.70710678, 0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_270            Matrix3f(0, 1, 0, -1, 0, 0, 0, 0, 1)
#define MATRIX_ROTATION_YAW_315            Matrix3f(0.70710678, 0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, 1)
#define MATRIX_ROTATION_ROLL_180           Matrix3f(1, 0, 0, 0, -1, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_45    Matrix3f(0.70710678, 0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_90    Matrix3f(0, 1, 0, 1, 0, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_135   Matrix3f(-0.70710678, 0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, -1)
#define MATRIX_ROTATION_PITCH_180          Matrix3f(-1, 0, 0, 0, 1, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_225   Matrix3f(-0.70710678, -0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_270   Matrix3f(0, -1, 0, -1, 0, 0, 0, 0, -1)
#define MATRIX_ROTATION_ROLL_180_YAW_315   Matrix3f(0.70710678, -0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, -1)

static void print_matrix(Matrix3f &m)
{
    Serial.printf("[%.2f %.2f %.2f] [%.2f %.2f %.2f] [%.2f %.2f %.2f]\n",
                  m.a.x, m.a.y, m.a.z,
                  m.b.x, m.b.y, m.b.z,
                  m.c.x, m.c.y, m.c.z);
}

// test one matrix
static void test_matrix(enum Rotation rotation, Matrix3f m)
{
	Matrix3f m2, diff;
	const float accuracy = 1.0e-6;
	m2.rotation(rotation);
	diff = (m - m2);
	if (diff.a.length() > accuracy ||
	    diff.b.length() > accuracy ||
	    diff.c.length() > accuracy) {
		Serial.printf("rotation matrix %u incorrect\n", (unsigned)rotation);
        print_matrix(m);
        print_matrix(m2);
	}
}

// test generation of rotation matrices
static void test_matrices(void)
{
    Serial.println("testing rotation matrices\n");
    test_matrix(ROTATION_NONE, MATRIX_ROTATION_NONE);
    test_matrix(ROTATION_YAW_45, MATRIX_ROTATION_YAW_45);
    test_matrix(ROTATION_YAW_90, MATRIX_ROTATION_YAW_90);
    test_matrix(ROTATION_YAW_135, MATRIX_ROTATION_YAW_135);
    test_matrix(ROTATION_YAW_180, MATRIX_ROTATION_YAW_180);
    test_matrix(ROTATION_YAW_225, MATRIX_ROTATION_YAW_225);
    test_matrix(ROTATION_YAW_270, MATRIX_ROTATION_YAW_270);
    test_matrix(ROTATION_YAW_315, MATRIX_ROTATION_YAW_315);
    test_matrix(ROTATION_ROLL_180, MATRIX_ROTATION_ROLL_180);
    test_matrix(ROTATION_ROLL_180_YAW_45, MATRIX_ROTATION_ROLL_180_YAW_45);
    test_matrix(ROTATION_ROLL_180_YAW_90, MATRIX_ROTATION_ROLL_180_YAW_90);
    test_matrix(ROTATION_ROLL_180_YAW_135, MATRIX_ROTATION_ROLL_180_YAW_135);
    test_matrix(ROTATION_PITCH_180, MATRIX_ROTATION_PITCH_180);
    test_matrix(ROTATION_ROLL_180_YAW_225, MATRIX_ROTATION_ROLL_180_YAW_225);
    test_matrix(ROTATION_ROLL_180_YAW_270, MATRIX_ROTATION_ROLL_180_YAW_270);
    test_matrix(ROTATION_ROLL_180_YAW_315, MATRIX_ROTATION_ROLL_180_YAW_315);
}

// test rotation of vectors
static void test_vector(enum Rotation rotation, Vector3f v1, bool show=true)
{
	Vector3f v2, diff;
	Matrix3f m;
	v2 = v1;
	m.rotation(rotation);
	v1.rotate(rotation);
	v2 = m * v2;
	diff = v1 - v2;
	if (diff.length() > 1.0e-6) {
		Serial.printf("rotation vector %u incorrect\n", (unsigned)rotation);
		Serial.printf("%u  %f %f %f\n",
			      (unsigned)rotation,
			      v2.x, v2.y, v2.z);
	}
    if (show) {
        Serial.printf("%u  %f %f %f\n",
                      (unsigned)rotation,
                      v1.x, v1.y, v1.z);
    }
}

// generate a random float between -1 and 1
static float rand_num(void)
{
	float ret = ((unsigned)random()) % 2000000;
	return (ret - 1.0e6) / 1.0e6;
}

// test rotation of vectors
static void test_vector(enum Rotation rotation)
{
    uint8_t i;

	Vector3f v1;
	v1.x = 1;
	v1.y = 2;
	v1.z = 3;
    test_vector(rotation, v1);

    for (i=0; i<10; i++) {
        v1.x = rand_num();
        v1.y = rand_num();
        v1.z = rand_num();
        test_vector(rotation, v1, false);
    }
}

// test rotation of vectors
static void test_vectors(void)
{
    Serial.println("testing rotation of vectors\n");
    test_vector(ROTATION_NONE);
    test_vector(ROTATION_YAW_45);
    test_vector(ROTATION_YAW_90);
    test_vector(ROTATION_YAW_135);
    test_vector(ROTATION_YAW_180);
    test_vector(ROTATION_YAW_225);
    test_vector(ROTATION_YAW_270);
    test_vector(ROTATION_YAW_315);
    test_vector(ROTATION_ROLL_180);
    test_vector(ROTATION_ROLL_180_YAW_45);
    test_vector(ROTATION_ROLL_180_YAW_90);
    test_vector(ROTATION_ROLL_180_YAW_135);
    test_vector(ROTATION_PITCH_180);
    test_vector(ROTATION_ROLL_180_YAW_225);
    test_vector(ROTATION_ROLL_180_YAW_270);
    test_vector(ROTATION_ROLL_180_YAW_315);
}


/*
  rotation tests
 */
void setup(void)
{
    Serial.begin(115200);
    Serial.println("rotation unit tests\n");
    test_matrices();
    test_vectors();
    Serial.println("rotation unit tests done\n");
}

void
loop(void)
{
}
