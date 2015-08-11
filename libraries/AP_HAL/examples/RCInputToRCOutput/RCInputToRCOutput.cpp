/*
    RC input pass trough to RC output
    Max RC channels 14
    Max update rate 10 Hz
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>
#include <Filter/Filter.h>
#include <AP_Param/AP_Param.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define MAX_CHANNELS 14

static uint8_t max_channels = 0;
static uint16_t last_value[MAX_CHANNELS];

void setup(void) 
{
    hal.console->printf("Starting RCInputToRCOutput test\n");

    for(uint8_t i = 0; i < MAX_CHANNELS; i++) {
        hal.rcout->enable_ch(i);
    }
}

void loop(void) 
{
    bool changed = false;
    uint8_t nchannels = hal.rcin->num_channels();

    if(nchannels > MAX_CHANNELS) {
       nchannels = MAX_CHANNELS;
    }

    for(uint8_t i = 0; i < nchannels; i++) {
        uint16_t v = hal.rcin->read(i);
        if(last_value[i] != v) {
            hal.rcout->write(i, v);
            changed = true;
            last_value[i] = v;
        }
        if(i > max_channels) {
            max_channels = i;
        }
    }
    if(changed) {
        for(uint8_t i = 0; i < max_channels; i++) {
            hal.console->printf("%2u:%04u ", (unsigned)i + 1, (unsigned)last_value[i]);
        }
        hal.console->println();
    }
    hal.scheduler->delay(100);
}

AP_HAL_MAIN();
