#include "AP_Baro_FAKE.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_Baro_FAKE::AP_Baro_FAKE(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    instance = _frontend.register_sensor();
}

AP_Baro_Backend *AP_Baro_FAKE::probe(AP_Baro &baro)
{
    return new AP_Baro_FAKE(baro);
}

// Read the sensor
void AP_Baro_FAKE::update(void)
{
    _copy_to_frontend(instance, fake_pressure, fake_temperature);
}
