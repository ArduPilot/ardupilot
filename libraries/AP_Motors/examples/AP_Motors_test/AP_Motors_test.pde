/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_NavEKF.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(rc1, rc2, rc3, rc4);
AP_MotorsQuad   motors(rc1, rc2, rc3, rc4);
//AP_MotorsHexa	motors(rc1, rc2, rc3, rc4);
//AP_MotorsY6	motors(rc1, rc2, rc3, rc4);
//AP_MotorsOcta	motors(rc1, rc2, rc3, rc4);
//AP_MotorsOctaQuad	motors(rc1, rc2, rc3, rc4);
//AP_MotorsHeli	motors(rc1, rc2, rc3, rc4);


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490);
    // motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    motors.set_min_throttle(130);
    motors.set_mid_throttle(500);
    motors.Init();      // initialise motors

    // setup radio
    if (rc3.radio_min == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_min = 1000;
    }
    if (rc3.radio_max == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_max = 2000;
    }
    // set rc channel ranges
    rc1.set_angle(4500);
    rc2.set_angle(4500);
    rc3.set_range(130, 1000);
    rc4.set_angle(4500);

    motors.enable();
    motors.output_min();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->println("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if (value == 't' || value == 'T') {
        motor_order_test();
    }
    if (value == 's' || value == 'S') {
        stability_test();
    }
}

// stability_test
void motor_order_test()
{
    hal.console->println("testing motor order");
    motors.armed(true);
    for (int8_t i=1; i <= AP_MOTORS_MAX_NUM_MOTORS; i++) {
        hal.console->printf_P(PSTR("Motor %d\n"),(int)i);
        motors.output_test(i, 1150);
        hal.scheduler->delay(300);
        motors.output_test(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);
    hal.console->println("finished test.");

}

// stability_test
void stability_test()
{
    int16_t value, roll_in, pitch_in, yaw_in, throttle_in, throttle_radio_in, avg_out;

    int16_t testing_array[][4] = {
        //  roll,   pitch,  yaw,    throttle
        {   0,      0,      0,      0},
        {   0,      0,      0,      100},
        {   0,      0,      0,      200},
        {   0,      0,      0,      300},
        {   4500,   0,      0,      300},
        {   -4500,  0,      0,      300},
        {   0,   4500,      0,      300},
        {   0,  -4500,      0,      300},
        {   0,      0,   4500,      300},
        {   0,      0,  -4500,      300},
        {   0,      0,      0,      400},
        {   0,      0,      0,      500},
        {   0,      0,      0,      600},
        {   0,      0,      0,      700},
        {   0,      0,      0,      800},
        {   0,      0,      0,      900},
        {   0,      0,      0,      1000},
        {   4500,   0,      0,      1000},
        {   -4500,  0,      0,      1000},
        {   0,   4500,      0,      1000},
        {   0,  -4500,      0,      1000},
        {   0,      0,   4500,      1000},
        {   0,      0,  -4500,      1000},
        {5000,   1000,      0,      1000},
        {5000,   2000,      0,      1000},
        {5000,   3000,      0,      1000},
        {5000,   4000,      0,      1000},
        {5000,   5000,      0,      1000},
        {5000,      0,   1000,      1000},
        {5000,      0,   2000,      1000},
        {5000,      0,   3000,      1000},
        {5000,      0,   4500,      1000}
    };
    uint32_t testing_array_rows = 32;

    hal.console->printf_P(PSTR("\nTesting stability patch\nThrottle Min:%d Max:%d\n"),(int)rc3.radio_min,(int)rc3.radio_max);

    // arm motors
    motors.armed(true);

    // run stability test
    for (int16_t i=0; i < testing_array_rows; i++) {
        roll_in = testing_array[i][0];
        pitch_in = testing_array[i][1];
        yaw_in = testing_array[i][2];
        throttle_in = testing_array[i][3];
        motors.set_pitch(roll_in);
        motors.set_roll(pitch_in);
        motors.set_yaw(yaw_in);
        motors.set_throttle(throttle_in);
        motors.output();
        // calc average output
        throttle_radio_in = rc3.radio_out;
        avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);

        // display input and output
        hal.console->printf_P(PSTR("R:%5d \tP:%5d \tY:%5d \tT:%5d\tMOT1:%5d \tMOT2:%5d \tMOT3:%5d \tMOT4:%5d \t ThrIn/AvgOut:%5d/%5d\n"),
                (int)roll_in,
                (int)pitch_in,
                (int)yaw_in,
                (int)throttle_in,
                (int)hal.rcout->read(0),
                (int)hal.rcout->read(1),
                (int)hal.rcout->read(2),
                (int)hal.rcout->read(3),
                (int)throttle_radio_in,
                (int)avg_out);
    }
    // set all inputs to motor library to zero and disarm motors
    motors.set_pitch(0);
    motors.set_roll(0);
    motors.set_yaw(0);
    motors.set_throttle(0);
    motors.armed(false);

    hal.console->println("finished test.");
}

AP_HAL_MAIN();
