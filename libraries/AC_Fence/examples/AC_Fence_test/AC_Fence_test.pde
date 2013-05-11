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
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <AP_InertialNav.h>     // Inertial Navigation library
#include <AC_Fence.h>           // Fence library
#include <GCS_MAVLink.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// INS and Baro declaration
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2

#define A_LED_PIN 27
#define C_LED_PIN 25
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

#else

#define A_LED_PIN 37
#define C_LED_PIN 35
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins(&adc);
AP_Baro_BMP085 baro;
#endif

// GPS declaration
GPS *gps;
AP_GPS_Auto auto_gps(&gps);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(&ins, gps);

// Inertial Nav declaration
AP_InertialNav inertial_nav(&ahrs, &ins, &baro, &gps);

// Fence
AC_Fence fence(&inertial_nav);

void setup()
{
    hal.console->println("AC_Fence library test");
}

void loop()
{
    // call update function
    hal.console->printf_P(PSTR("hello"));
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
