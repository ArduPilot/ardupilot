/*
 *       Example of AC_Fence library .
 *       DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_GPS.h>             // ArduPilot GPS library
#include <AP_GPS_Glitch.h>      // GPS glitch protection library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AC_PID.h>             // PID library
#include <AC_P.h>               // P library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_Fence.h>           // Fence library
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_InertialSensor ins;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
#endif

AP_Baro baro;

// GPS declaration
AP_GPS gps;
GPS_Glitch gps_glitch(gps);
Baro_Glitch baro_glitch(baro);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, baro, gps);

// Inertial Nav declaration
AP_InertialNav inertial_nav(ahrs, baro, gps_glitch, baro_glitch);

// Fence
AC_Fence fence(&inertial_nav);

void setup()
{
    hal.console->println("AC_Fence library test");
}

void loop()
{
    // print message to user
    hal.console->printf_P(PSTR("this example tests compilation only"));
    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
