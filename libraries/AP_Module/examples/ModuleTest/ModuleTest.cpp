//
// Simple test for the AP_AHRS interface
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/AP_Module.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// sensor declaration
static AP_InertialSensor ins;
#if HAL_EXTERNAL_AHRS_ENABLED
 static AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED
static AP_GPS gps;
static AP_Baro baro;
static AP_SerialManager serial_manager;

// choose which AHRS system to use
static AP_AHRS ahrs{};

void setup(void)
{
    serial_manager.init();
    ins.init(100);
    baro.init();
    ahrs.init();

    gps.init(serial_manager);
}

void loop(void)
{
    ahrs.update();
}

AP_HAL_MAIN();
