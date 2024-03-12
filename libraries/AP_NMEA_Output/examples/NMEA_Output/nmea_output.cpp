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
#include <AP_Vehicle/AP_Vehicle.h>

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

const struct AP_Param::Info var_info[] = {
    { "SERIAL", (const void *)&serial_manager, {group_info : AP_SerialManager::var_info}, 0, Parameters::k_param_serial_manager, AP_PARAM_GROUP },
    AP_VAREND
};


static AP_Param param{var_info};


AP_Int32 logger_bitmask;
static AP_Logger logger{logger_bitmask};

class DummyVehicle : public AP_Vehicle {
public:
    AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; };
    uint8_t get_mode() const override { return 1; };
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) override {};
    void init_ardupilot() override {};
    void load_parameters() override {};
    void init() {
        BoardConfig.init();
    }
};

static DummyVehicle vehicle;

void setup(void)
{
    vehicle.init();
    if (!AP_Param::setup()) {
        hal.console->printf("Failed to call setup\n");
        while(true);
    }
    if (!AP_Param::set_by_name("SERIAL0_PROTOCOL", AP_SerialManager::SerialProtocol_NMEAOutput)) {
        hal.console->printf("Failed to set SERIAL0_PROTOCOL\n");
        while(true);
    }
    AP::ins().init(100);
    serial_manager.init_console();
    serial_manager.init();
    vehicle.ahrs.init();

    AP::compass().init();
    if(!AP::compass().read()) {
        hal.console->printf("No compass detected\n");
    }
    AP::gps().init(serial_manager);
    AP::rtc().set_utc_usec(1546300800000, AP_RTC::source_type::SOURCE_GPS);
}

void loop(void)
{
    static uint32_t last_compass;
    const uint32_t now = AP_HAL::micros();

    // read compass at 10Hz
    if (now - last_compass > 100 * 1000UL &&
        AP::compass().read()) {
        last_compass = now;
    }

    vehicle.ahrs.update();
}

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
        AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
