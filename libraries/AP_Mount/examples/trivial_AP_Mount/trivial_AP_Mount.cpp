
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Buffer/AP_Buffer.h>
#include <Filter/Filter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#include <AP_Mount/AP_Mount.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup () {
    hal.console->println("Unit test for AP_Mount. This sketch"
                "has no functionality, it only tests build.");
}
void loop () {}

AP_HAL_MAIN();
