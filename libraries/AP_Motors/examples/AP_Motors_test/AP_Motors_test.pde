/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>

// Libraries
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <AP_Motors.h>
#include <AP_MotorsTri.h>
#include <AP_MotorsQuad.h>
#include <AP_MotorsHexa.h>
#include <AP_MotorsY6.h>
#include <AP_MotorsOcta.h>
#include <AP_MotorsOctaQuad.h>
#include <AP_MotorsHeli.h>
#include <AP_MotorsMatrix.h>

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console

Arduino_Mega_ISR_Registry isr_registry;

// uncomment one row below depending upon whether you have an APM1 or APM2
//APM_RC_APM1 APM_RC;
APM_RC_APM2 APM_RC;

RC_Channel rc1(CH_1), rc2(CH_2), rc3(CH_3), rc4(CH_4);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);
AP_MotorsQuad   motors(AP_MOTORS_APM2, &APM_RC, &rc1, &rc2, &rc3, &rc4);
//AP_MotorsHexa	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);
//AP_MotorsY6	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);
//AP_MotorsOcta	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);
//AP_MotorsOctaQuad	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);
//AP_MotorsHeli	motors(AP_MOTORS_APM1, &APM_RC, &rc1, &rc2, &rc3, &rc4);


// setup
void setup()
{
    Serial.begin(115200);
    Serial.println("AP_Motors library test ver 1.0");

    RC_Channel::set_apm_rc(&APM_RC);
    APM_RC.Init( &isr_registry );               // APM Radio initialization

    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.set_min_throttle(130);
    motors.set_max_throttle(850);
    motors.Init();      // initialise motors

    motors.enable();
    motors.output_min();

    delay(1000);
}

// loop
void loop()
{
    int value;

    // display help
    Serial.println("Press 't' to test motors.  Becareful they will spin!");

    // wait for user to enter something
    while( !Serial.available() ) {
        delay(20);
    }

    // get character from user
    value = Serial.read();

    // test motors
    if( value == 't' || value == 'T' ) {
        Serial.println("testing motors...");
        motors.armed(true);
        motors.output_test();
        motors.armed(false);
        Serial.println("finished test.");
    }
}

// print motor output
void print_motor_output()
{
    int8_t i;
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if( motors.motor_enabled[i] ) {
            Serial.printf("\t%d %d",i,motors.motor_out[i]);
        }
    }
}
