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
#include <StorageManager.h>
#include <AC_PID.h>
#include <AC_HELI_PID.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

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
    AC_HELI_PID heli_pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100);
    uint16_t radio_in;
    uint16_t radio_trim;
    int16_t error;
    float control_P, control_I, control_D;
    float dt = 1000/50;

    // display PID gains
    hal.console->printf("P %f  I %f  D %f  imax %f\n", (float)pid.kP(), (float)pid.kI(), (float)pid.kD(), (float)pid.imax());

    // capture radio trim
    radio_trim = hal.rcin->read(0);

    while( true ) {
        radio_in = hal.rcin->read(0);
        error = radio_in - radio_trim;
        control_P = pid.get_p(error);
        control_I = pid.get_i(error, dt);
        control_D = pid.get_d(error, dt);

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%4.2f (p:%4.2f i:%4.2f d:%4.2f)\n",
                (int)radio_in, (int)error,
                (float)(control_P+control_I+control_D),
                (float)control_P, (float)control_I, (float)control_D);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
