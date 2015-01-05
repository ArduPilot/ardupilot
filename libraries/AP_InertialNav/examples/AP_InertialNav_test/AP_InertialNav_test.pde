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
#include <AP_Baro_Glitch.h>     // Baro glitch protection library
#include <Filter.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Declination.h>
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AC_PID.h>             // PID library
#include <AC_P.h>               // P library
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Notify.h>
#include <AP_InertialNav.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_InertialSensor ins;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
#endif

AP_Baro baro;

AP_GPS gps;
GPS_Glitch   gps_glitch(gps);
Baro_Glitch baro_glitch(baro);

AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins, baro, gps);

AP_InertialNav inertialnav(ahrs, baro, gps_glitch, baro_glitch);

uint32_t last_update;

void setup(void)
{
    hal.console->println_P(PSTR("AP_InertialNav test startup..."));

    gps.init(NULL);

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);

    ahrs.set_compass(&compass);

    last_update = hal.scheduler->millis();

    inertialnav.init();
    inertialnav.set_velocity_xy(0,0);
    inertialnav.setup_home_position();
}

void loop(void)
{
    hal.scheduler->delay(20);
    gps.update();
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
