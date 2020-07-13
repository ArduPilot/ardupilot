//
// Simple test for the AP_AHRS NMEA output
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_NMEA_Output/AP_NMEA_Output.h>
#include <AP_SerialManager/AP_SerialManager.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();



class Parameters {
public:

    enum {
        k_param_serial_manager = 1, // serial manager library
    };
};

static AP_SerialManager serial_manager;

#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&v, {group_info : class::var_info} }
const struct AP_Param::Info var_info[] = {
//    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),
    { AP_PARAM_GROUP, "SERIAL", Parameters::k_param_serial_manager, (const void *)&serial_manager, {group_info : AP_SerialManager::var_info} },
    AP_VAREND
};

static AP_BoardConfig board_config;
static AP_InertialSensor ins;
static Compass compass;
static AP_Param param{var_info};
static AP_GPS gps;
static AP_Baro barometer;
AP_Int32 logger_bitmask;
static AP_Logger logger{logger_bitmask};

class DummyVehicle {
public:
    AP_AHRS_NavEKF ahrs{AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
};

static DummyVehicle vehicle;

void setup(void)
{
    board_config.init();
    if (!AP_Param::setup()) {
        hal.console->printf("Failed to call setup\n");
        while(true);
    }
    if (!AP_Param::set_by_name("SERIAL0_PROTOCOL", AP_SerialManager::SerialProtocol_NMEAOutput)) {
        hal.console->printf("Failed to set SERIAL0_PROTOCOL\n");
        while(true);
    }
    ins.init(100);
    serial_manager.init_console();
    serial_manager.init();
    vehicle.ahrs.init();

    compass.init();
    if(compass.read()) {
        hal.console->printf("Enabling compass\n");
        vehicle.ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(serial_manager);
    AP::rtc().set_utc_usec(1546300800000, AP_RTC::source_type::SOURCE_GPS);
}

void loop(void)
{
    static uint32_t last_compass;
    const uint32_t now = AP_HAL::micros();

    // read compass at 10Hz
    if (now - last_compass > 100 * 1000UL &&
        compass.read()) {
        last_compass = now;
    }

    vehicle.ahrs.update();
}

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
        AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
