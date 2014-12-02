/*
  simple test of RC input interface
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

#define MAX_CHANNELS 16

static uint8_t max_channels = 0;
static uint16_t last_value[MAX_CHANNELS];

void setup(void) 
{
    hal.console->printf("Starting RCInput test\n");
}

void loop(void) 
{
    bool changed = false;
    uint8_t nchannels = hal.rcin->num_channels();
    if (nchannels > MAX_CHANNELS) {
        nchannels = MAX_CHANNELS;
    }
    for (uint8_t i=0; i<nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if (last_value[i] != v) {
            changed = true;
            last_value[i] = v;
        }
        if (i > max_channels) {
            max_channels = i;
        }
    }
    if (changed) {
        for (uint8_t i=0; i<max_channels; i++) {
            hal.console->printf("%2u:%04u ", (unsigned)i+1, (unsigned)last_value[i]);
        }
        hal.console->println();
    }
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
