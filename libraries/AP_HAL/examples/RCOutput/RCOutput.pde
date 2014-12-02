/*
  simple test of RC output interface
 */
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <UARTDriver.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup (void) 
{
    hal.console->println("Starting AP_HAL::RCOutput test");
    for (uint8_t i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
    }
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop (void) 
{
    uint8_t i;
    for (i=0; i<14; i++) {
        hal.rcout->write(i, pwm);
        pwm += delta;
        if (delta > 0 && pwm >= 2000) {
            delta = -1;
            hal.console->printf("reversing\n");
        } else if (delta < 0 && pwm <= 1000) {
            delta = 1;
            hal.console->printf("reversing\n");
        }
    }
    hal.scheduler->delay(5);
}

AP_HAL_MAIN();
