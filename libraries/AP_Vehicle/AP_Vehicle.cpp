#include <AP_Vehicle/AP_Vehicle.h>

AP_Vehicle *AP_Vehicle::_singleton = nullptr;

AP_Vehicle *AP_Vehicle::get_singleton()
{
    return _singleton;
}


void AP_Vehicle::vehicle_setup(void)
{
#if HAL_HOTT_TELEM_ENABLED
    hott_telem.init();
#endif
}


namespace AP {

AP_Vehicle *vehicle()
{
    return AP_Vehicle::get_singleton();
}

};

