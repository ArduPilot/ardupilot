#include "AP_Baro_Dummy.h"

AP_Baro_Dummy::AP_Baro_Dummy(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

// Read the sensor
void AP_Baro_Dummy::update(void)
{
    _copy_to_frontend(0, 91300, 21);
}
