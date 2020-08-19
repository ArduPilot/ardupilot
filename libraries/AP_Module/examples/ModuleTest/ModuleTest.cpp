//
// Simple test for the AP_AHRS interface
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/AP_Module.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// sensor declaration
static AP_InertialSensor ins;
static AP_GPS gps;
static AP_Baro baro;
static AP_SerialManager serial_manager;

// choose which AHRS system to use
static AP_AHRS_DCM ahrs{};

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
