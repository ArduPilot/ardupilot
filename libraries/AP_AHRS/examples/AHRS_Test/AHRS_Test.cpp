//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// INS and Baro declaration
AP_InertialSensor ins;

Compass compass;

AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;

class DummyVehicle {
public:
    RangeFinder sonar {serial_manager, ROTATION_PITCH_270};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, sonar, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2{&ahrs, barometer, sonar};
    NavEKF3 EKF3{&ahrs, barometer, sonar};
};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM  ahrs(ins, baro, gps);
AP_AHRS_NavEKF ahrs(vehicle.ahrs);


#define HIGH 1
#define LOW 0

void setup(void)
{
    AP_BoardConfig{}.init();

    ins.init(100);
    ahrs.init();
    serial_manager.init();

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(nullptr, serial_manager);
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

    if (now - last_compass > 100*1000UL &&
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
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0f,
                        (1.0e6f*counter)/(now-last_print));
        last_print = now;
        counter = 0;
    }
}

GCS _gcs;

AP_HAL_MAIN();
