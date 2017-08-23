#include "SafeRTL_test.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_SerialManager/AP_SerialManager.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;

class DummyVehicle {
public:
    RangeFinder rangefinder {serial_manager, ROTATION_PITCH_270};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, rangefinder, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2{&ahrs, barometer, rangefinder};
    NavEKF3 EKF3{&ahrs, barometer, rangefinder};
};

static DummyVehicle vehicle;

AP_AHRS_NavEKF ahrs(vehicle.ahrs);
AP_SafeRTL safe_rtl {ahrs};

void setup();
void loop();
void reset();
bool check_path(const std::vector<Vector3f>&);

void setup()
{
    hal.console->printf("SafeRTL performance test\n");
    AP_BoardConfig{}.init();
    safe_rtl.init();
}

void loop()
{
    hal.scheduler->delay(5e3); // 5 seconds
    if (!hal.console->is_initialized()) {
        return;
    }
    uint32_t reference_time, run_time;
    bool correct;

    hal.console->printf("--------------------\n");

    // test reset_path() and update()
    reset();
    correct = check_path(test_path_after_adding);
    hal.console->printf("append: %s\n", correct ? "success" : "fail");

    // test detect_simplifications()
    reference_time = AP_HAL::micros();
    for (uint16_t i = 0; i < 1000; i++) {
        safe_rtl.detect_simplifications();
    }
    run_time = AP_HAL::micros() - reference_time;
    safe_rtl.thorough_cleanup();
    correct = check_path(test_path_after_simplifying);
    hal.console->printf("rdp:    %s, %u usec\n", correct ? "success" : "fail", run_time);

    // test detect_loops()
    reset();
    reference_time = AP_HAL::micros();
    for (uint16_t i = 0; i < 1000; i++) {
        safe_rtl.detect_loops();
    }
    run_time = AP_HAL::micros() - reference_time;
    safe_rtl.thorough_cleanup();
    correct = check_path(test_path_after_pruning);
    hal.console->printf("prune:  %s, %u usec\n", correct ? "success" : "fail", run_time);

    // test both
    reset();
    reference_time = AP_HAL::micros();
    while (!(safe_rtl.thorough_cleanup())) {
        hal.scheduler->delay(10);
    }
    run_time = AP_HAL::micros() - reference_time;
    correct = check_path(test_path_complete);
    hal.console->printf("both:   %s, %u usec\n", correct ? "success" : "fail", run_time);
}

void reset()
{
    safe_rtl.reset_path(true, Vector3f{0.0f, 0.0f, 0.0f});
    for (Vector3f v : test_path_before) {
        safe_rtl.update(true, v);
    }
}

bool check_path(const std::vector<Vector3f>& correct)
{
    for (uint16_t i = 0; i < correct.size(); i++) {
        if (!is_equal(safe_rtl.get_point(i)[0],correct[i][0]) || !is_equal(safe_rtl.get_point(i)[1],correct[i][1])|| !is_equal(safe_rtl.get_point(i)[2],correct[i][2])) {
            return false;
        }
    }
    return true;
}

AP_HAL_MAIN();
