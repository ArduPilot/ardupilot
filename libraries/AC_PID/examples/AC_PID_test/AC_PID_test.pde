/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AC_PID.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

// default PID values
#define TEST_P 1.0
#define TEST_I 0.01
#define TEST_D 0.2
#define TEST_IMAX 10

// setup function
void setup()
{
    hal.console->println("ArduPilot Mega AC_PID library test");

    hal.scheduler->delay(1000);
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
    hal.console->printf("P %f  I %f  D %f  imax %f\n", pid.kP(), pid.kI(), pid.kD(), pid.imax());

    // capture radio trim
    radio_trim = hal.rcin->read(0);

    while( true ) {
        radio_in = hal.rcin->read(0);
        error = radio_in - radio_trim;
        control = pid.get_pid(error, dt);

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%d\n", radio_in, error, control);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
