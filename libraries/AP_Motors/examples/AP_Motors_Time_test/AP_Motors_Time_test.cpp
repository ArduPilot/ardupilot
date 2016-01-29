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

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(rc1, rc2, rc3, rc4, 400);
AP_MotorsQuad   motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsHexa	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsY6	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsOcta	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsOctaQuad	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsHeli	motors(rc1, rc2, rc3, rc4, 400);


// setup
void setup()
{
    hal.console->println("AP_Motors_Time test");

    // motor initialisation
    motors.set_update_rate(490);
    // motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    motors.set_min_throttle(130);
    motors.set_hover_throttle(500);
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

    motor_order_test();
}

// stability_test
void motor_order_test()
{

    motors.armed(true);
    for (int8_t i=1; i <= 4; i++) {
		hal.console->printf("Motor %d\n",(int)i);
        int elapsed =0,stop;
		int start = AP_HAL::micros();                                                   //Time Test
        motors.output_test(i, 1150);
        stop = AP_HAL::micros();
        elapsed = stop - start;
        hal.console->printf("  Elapsed Time: %dus\n",elapsed);
        hal.scheduler->delay(300);
        motors.output_test(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors.armed(false);
   
   hal.console->printf("\n\n");
}

AP_HAL_MAIN();
