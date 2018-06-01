

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro/AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass/AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC/AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID/PID.h>            // PID library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_RangeFinder/AP_RangeFinder.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Buffer/AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay/AP_Relay.h>       // APM relay
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL_AVR/memcheck.h>

#include <DataFlash/DataFlash.h>
#include <APM_Control/APM_Control.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_BattMonitor/AP_BattMonitor.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif


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

    for(;;);

}


void loop(void){}

AP_HAL_MAIN();
