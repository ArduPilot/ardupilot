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

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


static AP_BoardConfig board_config;
static AP_InertialSensor ins;

static Compass compass;

static AP_GPS gps;
static AP_Baro barometer;
static AP_SerialManager serial_manager;
AP_Int32 logger_bitmask;
static AP_Logger logger{logger_bitmask};

class DummyVehicle {
public:
    RangeFinder sonar;
    NavEKF2 EKF2{&ahrs, sonar};
    NavEKF3 EKF3{&ahrs, sonar};
    AP_AHRS_NavEKF ahrs{EKF2, EKF3,
            AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM ahrs = AP_AHRS_DCM::create(barometer, gps);
AP_AHRS_NavEKF &ahrs = vehicle.ahrs;

void setup(void)
{
    board_config.init();
    ins.init(100);
    ahrs.init();
    serial_manager.init();

    compass.init();
    if(compass.read()) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(serial_manager);
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
        compass.read()) {
        heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned());
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
                (double)(compass.use_for_yaw() ? ToDeg(heading) : 0.0f),
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
