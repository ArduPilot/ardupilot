/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 */

// includes
#include <Arduino_Mega_ISR_Registry.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <APM_RC.h> // ArduPilot RC Library
#include <AC_PID.h> // ArduPilot Mega RC Library

// default PID values
#define TEST_P 1.0
#define TEST_I 0.01
#define TEST_D 0.2
#define TEST_IMAX 10

// Serial ports
FastSerialPort0(Serial);        // FTDI/console

// Global variables
Arduino_Mega_ISR_Registry isr_registry;
APM_RC_APM1 APM_RC;

// setup function
void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot Mega AC_PID library test");

    isr_registry.init();
    APM_RC.Init(&isr_registry);          // APM Radio initialization

    delay(1000);
}

// main loop
void loop()
{
    // setup (unfortunately must be done here as we cannot create a global AC_PID object)
    AC_PID pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100);
    uint16_t radio_in;
    uint16_t radio_trim;
    int error;
    int control;
    float dt = 1000/50;

    // display PID gains
    Serial.printf("P %f  I %f  D %f  imax %f\n", pid.kP(), pid.kI(), pid.kD(), pid.imax());

    // capture radio trim
    radio_trim = APM_RC.InputCh(0);

    while( true ) {
        radio_in = APM_RC.InputCh(0);
        error = radio_in - radio_trim;
        control = pid.get_pid(error, dt);

        // display pid results
        Serial.printf("radio: %d\t err: %d\t pid:%d\n", radio_in, error, control);
        delay(50);
    }
}
