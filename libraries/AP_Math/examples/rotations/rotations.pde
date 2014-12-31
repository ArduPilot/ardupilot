/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math rotations code
//
#include <AP_HAL.h>
#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_Math.h>
#include <Filter.h>
#include <AP_ADC.h>
#include <SITL.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <AP_Notify.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Rally.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static void print_vector(Vector3f &v)
{
    hal.console->printf("[%.4f %.4f %.4f]\n",
                        v.x, v.y, v.z);
}

// test rotation method accuracy
static void test_rotation_accuracy(void)
{
    Matrix3f attitude;
    Vector3f small_rotation;
    float roll, pitch, yaw;
    int16_t i;
    float rot_angle;

    hal.console->println_P(PSTR("\nRotation method accuracy:"));

    for( i=0; i<90; i++ ) {

        // reset initial attitude
        attitude.from_euler(0,0,0);

        // calculate small rotation vector
        rot_angle = ToRad(i);
        small_rotation = Vector3f(0,0,rot_angle);

        // apply small rotation
        attitude.rotate(small_rotation);

        // get resulting attitude's euler angles
        attitude.to_euler(&roll, &pitch, &yaw);

        // display results
        hal.console->printf_P(
                PSTR("actual angle: %d\tcalculated angle:%4.2f\n"),
                (int)i,ToDeg(yaw));
    }
}

static void test_euler(enum Rotation rotation, float roll, float pitch, float yaw)
{
    Vector3f v, v1, v2, diff;
    Matrix3f rotmat;
    const float accuracy = 1.0e-6;

    v.x = 1;
    v.y = 2;
    v.z = 3;
    v1 = v;

    v1.rotate(rotation);
    
    rotmat.from_euler(radians(roll), radians(pitch), radians(yaw));
    v2 = v;
    v2 = rotmat * v2;

    diff = (v2 - v1);
    if (diff.length() > accuracy) {
        hal.console->printf("euler test %u failed : yaw:%d roll:%d pitch:%d\n",
        (unsigned)rotation,
        (int)yaw,
        (int)roll,
        (int)pitch);
        hal.console->printf("fast rotated: ");
        print_vector(v1);
        hal.console->printf("slow rotated: ");
        print_vector(v2);
        hal.console->printf("\n");
    }
}

static void test_eulers(void)
{
    hal.console->println("euler tests");
    test_euler(ROTATION_NONE,               0,   0,   0);
    test_euler(ROTATION_YAW_45,             0,   0,  45);
    test_euler(ROTATION_YAW_90,             0,   0,  90);
    test_euler(ROTATION_YAW_135,            0,   0, 135);
    test_euler(ROTATION_YAW_180,            0,   0, 180);
    test_euler(ROTATION_YAW_225,            0,   0, 225);
    test_euler(ROTATION_YAW_270,            0,   0, 270);
    test_euler(ROTATION_YAW_315,            0,   0, 315);
    test_euler(ROTATION_ROLL_180,         180,   0,   0);
    test_euler(ROTATION_ROLL_180_YAW_45,  180,   0,  45);
    test_euler(ROTATION_ROLL_180_YAW_90,  180,   0,  90);
    test_euler(ROTATION_ROLL_180_YAW_135, 180,   0, 135);
    test_euler(ROTATION_PITCH_180,          0, 180,   0);
    test_euler(ROTATION_ROLL_180_YAW_225, 180,   0, 225);
    test_euler(ROTATION_ROLL_180_YAW_270, 180,   0, 270);
    test_euler(ROTATION_ROLL_180_YAW_315, 180,   0, 315);
    test_euler(ROTATION_ROLL_90,           90,   0,   0);
    test_euler(ROTATION_ROLL_90_YAW_45,    90,   0,  45);
    test_euler(ROTATION_ROLL_90_YAW_90,    90,   0,  90);
    test_euler(ROTATION_ROLL_90_YAW_135,   90,   0, 135);
    test_euler(ROTATION_ROLL_270,         270,   0,   0);
    test_euler(ROTATION_ROLL_270_YAW_45,  270,   0,  45);
    test_euler(ROTATION_ROLL_270_YAW_90,  270,   0,  90);
    test_euler(ROTATION_ROLL_270_YAW_135, 270,   0, 135);
    test_euler(ROTATION_PITCH_90,           0,  90,   0);
    test_euler(ROTATION_PITCH_270,          0, 270,   0);    
    test_euler(ROTATION_PITCH_180_YAW_90,   0, 180,  90);    
    test_euler(ROTATION_PITCH_180_YAW_270,  0, 180, 270);    
    test_euler(ROTATION_ROLL_90_PITCH_90,  90,  90,   0);    
    test_euler(ROTATION_ROLL_180_PITCH_90,180,  90,   0);    
    test_euler(ROTATION_ROLL_270_PITCH_90,270,  90,   0);    
    test_euler(ROTATION_ROLL_90_PITCH_180, 90, 180,   0);    
    test_euler(ROTATION_ROLL_270_PITCH_180,270,180,   0);    
    test_euler(ROTATION_ROLL_90_PITCH_270, 90, 270,   0);    
    test_euler(ROTATION_ROLL_180_PITCH_270,180,270,   0);    
    test_euler(ROTATION_ROLL_270_PITCH_270,270,270,   0);    
    test_euler(ROTATION_ROLL_90_PITCH_180_YAW_90, 90, 180,  90);    
    test_euler(ROTATION_ROLL_90_YAW_270,   90,   0, 270);
    test_euler(ROTATION_YAW_293_PITCH_68_ROLL_90,90,68.8,293.3);
}

static bool have_rotation(const Matrix3f &m)
{
    Matrix3f mt = m.transposed();
    for (enum Rotation r=ROTATION_NONE; 
         r<ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        Vector3f v(1,2,3);
        Vector3f v2 = v;
        v2.rotate(r);
        v2 = mt * v2;
        if ((v2 - v).length() < 0.01f) {
            return true;
        }
    }
    return false;
}

static void missing_rotations(void)
{
    hal.console->println("testing for missing rotations");
    uint16_t roll, pitch, yaw;
    for (yaw=0; yaw<360; yaw += 90)
        for (pitch=0; pitch<360; pitch += 90)
            for (roll=0; roll<360; roll += 90) {
                Matrix3f m;
                m.from_euler(ToRad(roll), ToRad(pitch), ToRad(yaw));
                if (!have_rotation(m)) {
                    hal.console->printf("Missing rotation (%u, %u, %u)\n", roll, pitch, yaw);
                }
            }
}

/*
 *  rotation tests
 */
void setup(void)
{
    hal.console->println("rotation unit tests\n");
    test_rotation_accuracy();
    test_eulers();
    missing_rotations();
    hal.console->println("rotation unit tests done\n");
}

void loop(void) {}

AP_HAL_MAIN();
