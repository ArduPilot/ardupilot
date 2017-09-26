//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/AP_Module.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// sensor declaration
static AP_InertialSensor ins = AP_InertialSensor::create();
static AP_GPS gps = AP_GPS::create();
static AP_Baro baro = AP_Baro::create();
static AP_SerialManager serial_manager = AP_SerialManager::create();

// choose which AHRS system to use
static AP_AHRS_DCM ahrs = AP_AHRS_DCM::create(ins, baro, gps);

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
