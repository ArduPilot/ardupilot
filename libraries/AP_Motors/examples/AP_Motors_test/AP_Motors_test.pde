/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>

#include <AP_HAL_AVR.h>
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(&rc1, &rc2, &rc3, &rc4);
AP_MotorsQuad   motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHexa	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsY6	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOcta	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsOctaQuad	motors(&rc1, &rc2, &rc3, &rc4);
//AP_MotorsHeli	motors(&rc1, &rc2, &rc3, &rc4);


// setup
void setup()
{
    hal.console->println("AP_Motors library test ver 1.0");

    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_min_throttle(130);
    motors.Init();      // initialise motors

    if (rc3.radio_min == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_min = 1000;
    }
    if (rc3.radio_max == 0) {
	    // cope with AP_Param not being loaded
	    rc3.radio_max = 2000;
    }

    motors.enable();
    motors.output_min();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int value;

    // display help
    hal.console->println("Press 't' to test motors.  Be careful they will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if( value == 't' || value == 'T' ) {
        hal.console->println("testing motors...");
        motors.armed(true);
        motors.output_test();
        motors.armed(false);
        hal.console->println("finished test.");
    }
}

// print motor output
void print_motor_output()
{
    int8_t i;
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if( motors.motor_enabled[i] ) {
            hal.console->printf("\t%d %d",i,motors.motor_out[i]);
        }
    }
}

AP_HAL_MAIN();
