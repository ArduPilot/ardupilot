/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyVehicle {
public:
    AP_GPS gps = AP_GPS::create();
    AP_Baro barometer = AP_Baro::create();
    Compass compass = Compass::create();
    AP_InertialSensor ins = AP_InertialSensor::create();
    AP_SerialManager serial_manager = AP_SerialManager::create();
    RangeFinder sonar = RangeFinder::create(serial_manager, ROTATION_PITCH_270);
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2 = NavEKF2::create(&ahrs, barometer, sonar);
    NavEKF3 EKF3 = NavEKF3::create(&ahrs, barometer, sonar);
};

static DummyVehicle vehicle;
static OpticalFlow optflow = OpticalFlow::create(vehicle.ahrs);

void setup()
{
    hal.console->printf("OpticalFlow library test ver 1.6\n");

    hal.scheduler->delay(1000);

    // flowSensor initialization
    optflow.init();

    if (!optflow.healthy()) {
        hal.console->printf("Failed to initialise PX4Flow ");
    }

    hal.scheduler->delay(1000);
}

void loop()
{
    hal.console->printf("this only tests compilation succeeds\n");

    hal.scheduler->delay(5000);
}

AP_HAL_MAIN();
