/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void motor_order_test();
void stability_test();
void update_motors();

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri  motors(400);
AP_MotorsQuad   motors(400);
//AP_MotorsHexa motors(400);
//AP_MotorsY6   motors(400);
//AP_MotorsOcta motors(400);
//AP_MotorsOctaQuad	motors(400);
//AP_MotorsHeli	motors(rc1, rc2, rc3, rc4, 400);


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.Init();
    motors.set_throttle_range(130,1000,2000);
    motors.set_hover_throttle(500);
    motors.enable();
    motors.output_min();

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
        hal.console->printf("Motor %d\n",(int)i);
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
    int16_t roll_in, pitch_in, yaw_in, throttle_in, avg_out;

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

    hal.console->printf("\nTesting stability patch\nThrottle Min:%d Max:%d\n",(int)rc3.radio_min,(int)rc3.radio_max);

    // arm motors
    motors.armed(true);
    motors.set_interlock(true);
    motors.set_stabilizing(true);

    // run stability test
    for (uint16_t i=0; i<testing_array_rows; i++) {
        roll_in = testing_array[i][0];
        pitch_in = testing_array[i][1];
        yaw_in = testing_array[i][2];
        throttle_in = testing_array[i][3];
        motors.set_pitch(roll_in);
        motors.set_roll(pitch_in);
        motors.set_yaw(yaw_in);
        motors.set_throttle(throttle_in);
        update_motors();
        // calc average output
        avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);

        // display input and output
        hal.console->printf("R:%5d \tP:%5d \tY:%5d \tT:%5d\tMOT1:%5d \tMOT2:%5d \tMOT3:%5d \tMOT4:%5d \t AvgOut:%5d\n",
                (int)roll_in,
                (int)pitch_in,
                (int)yaw_in,
                (int)throttle_in,
                (int)hal.rcout->read(0),
                (int)hal.rcout->read(1),
                (int)hal.rcout->read(2),
                (int)hal.rcout->read(3),
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

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting completed
    for (uint16_t i=0; i<1000; i++) {
        motors.output();
    }
}

AP_HAL_MAIN();
