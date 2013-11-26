// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
#include <DataFlash.h>
#include <AC_PID.h>             // PID library
#include <APM_PI.h>             // PID library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <GCS_MAVLink.h>
#include <AP_Notify.h>

#include <AP_InertialNav.h>
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2

AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

#else

AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins(&adc);
AP_Baro_BMP085 baro;
#endif

GPS *gps;
AP_GPS_Auto auto_gps(&gps);
GPS_Glitch   gps_glitch(gps);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, gps);

AP_InertialNav inertialnav(&ahrs, &baro, gps, gps_glitch);

uint32_t last_update;

void setup(void)
{
    hal.console->println_P(PSTR("AP_InertialNav test startup..."));

    gps = &auto_gps;
    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);

    ahrs.set_compass(&compass);

    last_update = hal.scheduler->millis();

    inertialnav.init();
    inertialnav.set_velocity_xy(0,0);
    inertialnav.set_home_position(0,0);
}

void loop(void)
{
    hal.scheduler->delay(20);
    gps->update();
    ahrs.update();
    uint32_t currtime = hal.scheduler->millis();
    float dt = (currtime - last_update) / 1000.0f;
    last_update = currtime;
    inertialnav.update(dt);

    float dx =  inertialnav.get_latitude_diff();
    float dy =  inertialnav.get_longitude_diff();

    hal.console->printf_P(
            PSTR("inertial nav pos: (%f,%f)\r\n"),
            dx, dy);
}

AP_HAL_MAIN();
