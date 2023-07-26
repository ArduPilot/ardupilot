//
// Simple test for the AP_AHRS interface
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


static AP_SerialManager serial_manager;
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
        ins.init(100);
        ahrs.init();
    }

};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM ahrs = AP_AHRS_DCM::create(barometer, gps);
auto &ahrs = vehicle.ahrs;

void setup(void)
{
    vehicle.init();
    serial_manager.init();
    AP::compass().init();
    if (!AP::compass().read()) {
        hal.console->printf("No compass detected\n");
    }
    AP::gps().init(serial_manager);
}

void loop(void)
{
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = AP_HAL::micros();
    float heading = 0;

    if (last_t == 0) {
        last_t = now;
        return;
    }
    last_t = now;

    if (now - last_compass > 100 * 1000UL &&
        AP::compass().read()) {
        heading = AP::compass().calculate_heading(ahrs.get_rotation_body_to_ned());
        // read compass at 10Hz
        last_compass = now;
    }

    ahrs.update();
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf(
                "r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n",
                (double)ToDeg(ahrs.roll),
                (double)ToDeg(ahrs.pitch),
                (double)ToDeg(ahrs.yaw),
                (double)ToDeg(drift.x),
                (double)ToDeg(drift.y),
                (double)ToDeg(drift.z),
                (double)(AP::compass().use_for_yaw() ? ToDeg(heading) : 0.0f),
                (double)((1.0e6f * counter) / (now-last_print)));
        last_print = now;
        counter = 0;
    }
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
