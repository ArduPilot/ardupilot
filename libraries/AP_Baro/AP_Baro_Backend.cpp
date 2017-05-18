#include "AP_Baro_Backend.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Baro_Backend::AP_Baro_Backend(AP_Baro &baro) : 
    _frontend(baro) 
{
    _sem = hal.util->new_semaphore();    
}

/*
  copy latest data to the frontend from a backend
 */
void AP_Baro_Backend::_copy_to_frontend(uint8_t instance, float pressure, float temperature)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    _frontend.sensors[instance].pressure = pressure;
    _frontend.sensors[instance].temperature = temperature;
    _frontend.sensors[instance].last_update_ms = AP_HAL::millis();
}
