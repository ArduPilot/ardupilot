
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>             // ArduPilot GPS library
#include <AP_ADC/AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_Baro/AP_Baro.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Compass/AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve/AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
                                // (only included for makefile libpath to work)
#include <AP_AHRS/AP_AHRS.h>
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_P.h>               // P library
#include <RC_Channel/RC_Channel.h>         // RC Channel Library
#include <AP_Motors/AP_Motors.h>          // AP Motors library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <Filter/Filter.h>             // Filter library
#include <AP_Buffer/AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay/AP_Relay.h>           // APM relay
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed/AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialNav/AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_HAL_AVR/memcheck.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif


void stream_loopback(AP_HAL::Stream* s, uint32_t time) {
    uint32_t end = hal.scheduler->millis() + time;
    for(;;) {
        if (hal.scheduler->millis() >= end && time != 0) {
            return;
        }
        if (s->available() > 0) {
            int c;
            c = s->read();
            if (-1 != c) {
                s->write((uint8_t)c);
            }
        }
    }
}

void setup(void)
{
        //
        // Test printing things
        //
    hal.console->print("test");
    hal.console->println(" begin");
    hal.console->println(1000);
    hal.console->println(1000, 8);
    hal.console->println(1000, 10);
    hal.console->println(1000, 16);
    hal.console->println_P(PSTR("progmem"));
    hal.console->printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
    hal.console->printf_P(PSTR("printf_P %d %u %#x %p %f %S\n"), -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
    hal.console->println("done.");
    for(;;);

}


void loop(void){}

AP_HAL_MAIN();
