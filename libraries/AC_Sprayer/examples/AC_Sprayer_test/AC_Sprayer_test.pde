/*
 *       Example of AC_WPNav library .
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
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>
#include <AC_Sprayer.h>         // Crop Sprayer library
#include <AP_Notify.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// INS and Baro declaration
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2

AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

#else

AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins(&adc);
AP_Baro_BMP085 baro;
#endif

// GPS declaration
GPS *gps;
AP_GPS_Auto auto_gps(&gps);
GPS_Glitch gps_glitch(gps);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, gps);

// Inertial Nav declaration
AP_InertialNav inertial_nav(&ahrs, &baro, gps, gps_glitch);

// Sprayer
AC_Sprayer sprayer(&inertial_nav);

void setup()
{
    // display welcome message
    hal.console->println("AC_Sprayer library test");

    // enable sprayer
    sprayer.enable(true);

    // set-up pump rate
    sprayer.set_pump_rate(1.0);
}

void loop()
{
    // repeated call sprayer at different velocities
    sprayer.update();
}

AP_HAL_MAIN();
