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
AP_InertialSensor ins;
AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, gps);

void setup(void)
{
    serial_manager.init();
    ins.init(100);
    baro.init();
    ahrs.init();

    gps.init(nullptr, serial_manager);
}

void loop(void)
{
    ahrs.update();
}

AP_HAL_MAIN();
