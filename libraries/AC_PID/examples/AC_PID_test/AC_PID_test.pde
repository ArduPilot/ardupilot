/*
 *       Example of PID library.
 *       2012 Code by Jason Short, Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <GCS_MAVLink.h>
#include <StorageManager.h>
#include <AC_PID.h>
#include <AC_HELI_PID.h>
#include <AP_Scheduler.h>
#include <DataFlash.h>
#include <AP_GPS.h>
#include <AP_Vehicle.h>
#include <AP_InertialSensor.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_NavEKF.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Notify.h>
#include <AP_Mission.h>
#include <AP_Terrain.h>
#include <AP_Rally.h>
#include <AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// default PID values
#define TEST_P 1.0f
#define TEST_I 0.01f
#define TEST_D 0.2f
#define TEST_IMAX 10
#define TEST_FILTER 5.0f
#define TEST_DT 0.01f

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
    AC_PID pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100, TEST_FILTER, TEST_DT);
    AC_HELI_PID heli_pid(TEST_P, TEST_I, TEST_D, TEST_IMAX * 100, TEST_FILTER, TEST_DT);
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
        pid.set_input_filter_all(error);
        control_P = pid.get_p();
        control_I = pid.get_i();
        control_D = pid.get_d();

        // display pid results
        hal.console->printf("radio: %d\t err: %d\t pid:%4.2f (p:%4.2f i:%4.2f d:%4.2f)\n",
                (int)radio_in, (int)error,
                (float)(control_P+control_I+control_D),
                (float)control_P, (float)control_I, (float)control_D);
        hal.scheduler->delay(50);
    }
}

AP_HAL_MAIN();
