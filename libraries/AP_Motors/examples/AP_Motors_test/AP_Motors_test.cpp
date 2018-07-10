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
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void motor_order_test();
void stability_test();
void update_motors();

#define HELI_TEST       0   // set to 1 to test helicopters
#define NUM_OUTPUTS     4   // set to 6 for hexacopter, 8 for octacopter and heli

SRV_Channels srvs;

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(400);
AP_MotorsMatrix   motors(400);
//AP_MotorsHeli_Single motors(rc7, rsc, h1, h2, h3, h4, 400);
//AP_MotorsSingle motors(400);
//AP_MotorsCoax motors(400);

// setup
void setup()
{
    hal.console->printf("AP_Motors library test ver 1.0\n");

    // motor initialisation
    motors.set_update_rate(490);
    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
#if HELI_TEST == 0
    motors.set_throttle_range(1000,2000);
    motors.set_throttle_avg_max(0.5f);
#endif
    motors.output_min();

    // setup radio
    SRV_Channels::srv_channel(2)->set_output_min(1000);
    SRV_Channels::srv_channel(2)->set_output_max(2000);

    // set rc channel ranges
    SRV_Channels::srv_channel(0)->set_angle(4500);
    SRV_Channels::srv_channel(1)->set_angle(4500);
    SRV_Channels::srv_channel(2)->set_range(1000);
    SRV_Channels::srv_channel(3)->set_angle(4500);

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");

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
    hal.console->printf("testing motor order\n");
    motors.armed(true);
    for (int8_t i=1; i <= AP_MOTORS_MAX_NUM_MOTORS; i++) {
        hal.console->printf("Motor %d\n",(int)i);
        motors.output_test_seq(i, 1150);
        hal.scheduler->delay(300);
        motors.output_test_seq(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);
    hal.console->printf("finished test.\n");

}

// stability_test
void stability_test()
{
    int16_t roll_in, pitch_in, yaw_in, avg_out;
    float throttle_in;

    int16_t throttle_tests[] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
    uint8_t throttle_tests_num = sizeof(throttle_tests) / sizeof(int16_t);
    int16_t rpy_tests[] = {0, 1000, 2000, 3000, 4500, -1000, -2000, -3000, -4500};
    uint8_t rpy_tests_num = sizeof(rpy_tests) / sizeof(int16_t);

    hal.console->printf("\nTesting stability patch\nThrottle Min:%d Max:%d\n",(int)SRV_Channels::srv_channel(2)->get_output_min(),(int)SRV_Channels::srv_channel(2)->get_output_max());

    // arm motors
    motors.armed(true);
    motors.set_interlock(true);
    SRV_Channels::enable_aux_servos();

#if NUM_OUTPUTS <= 4
    hal.console->printf("Roll,Pitch,Yaw,Thr,Mot1,Mot2,Mot3,Mot4,AvgOut,LimRP,LimY,LimThD,LimThU\n");                       // quad
#elif NUM_OUTPUTS <= 6
    hal.console->printf("Roll,Pitch,Yaw,Thr,Mot1,Mot2,Mot3,Mot4,Mot5,Mot6,AvgOut,LimRP,LimY,LimThD,LimThU\n");             // hexa
#else
    hal.console->printf("Roll,Pitch,Yaw,Thr,Mot1,Mot2,Mot3,Mot4,Mot5,Mot6,Mot7,Mot8,AvgOut,LimRP,LimY,LimThD,LimThU\n");   // octa
#endif

    // run stability test
    for (uint8_t y=0; y<rpy_tests_num; y++) {
        for (uint8_t p=0; p<rpy_tests_num; p++) {
            for (uint8_t r=0; r<rpy_tests_num; r++) {
                for (uint8_t t=0; t<throttle_tests_num; t++) {
                    roll_in = rpy_tests[r];
                    pitch_in = rpy_tests[p];
                    yaw_in = rpy_tests[y];
                    throttle_in = throttle_tests[t]/1000.0f;
                    motors.set_roll(roll_in/4500.0f);
                    motors.set_pitch(pitch_in/4500.0f);
                    motors.set_yaw(yaw_in/4500.0f);
                    motors.set_throttle(throttle_in);
                    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
                    update_motors();
                    avg_out = ((hal.rcout->read(0) + hal.rcout->read(1) + hal.rcout->read(2) + hal.rcout->read(3))/4);
                    // display input and output
#if NUM_OUTPUTS <= 4
                    hal.console->printf("%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",                // quad
#elif NUM_OUTPUTS <= 6
                    hal.console->printf("%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",          // hexa
#else
                    hal.console->printf("%d,%d,%d,%3.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",    // octa
#endif
                            (int)roll_in,
                            (int)pitch_in,
                            (int)yaw_in,
                            (double)throttle_in,
                            (int)hal.rcout->read(0),
                            (int)hal.rcout->read(1),
                            (int)hal.rcout->read(2),
                            (int)hal.rcout->read(3),
#if NUM_OUTPUTS >= 6
                            (int)hal.rcout->read(4),
                            (int)hal.rcout->read(5),
#endif
#if NUM_OUTPUTS >= 8
                            (int)hal.rcout->read(6),
                            (int)hal.rcout->read(7),
#endif
                            (int)avg_out,
                            (int)motors.limit.roll_pitch,
                            (int)motors.limit.yaw,
                            (int)motors.limit.throttle_lower,
                            (int)motors.limit.throttle_upper);
                }
            }
        }
    }

    // set all inputs to motor library to zero and disarm motors
    motors.set_pitch(0);
    motors.set_roll(0);
    motors.set_yaw(0);
    motors.set_throttle(0);
    motors.armed(false);

    hal.console->printf("finished test.\n");
}

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting complete
    for (uint16_t i=0; i<1000; i++) {
        motors.output();
    }
}

AP_HAL_MAIN();
