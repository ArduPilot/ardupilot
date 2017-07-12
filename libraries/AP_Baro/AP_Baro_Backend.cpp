#include "AP_Baro_Backend.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Baro_Backend::AP_Baro_Backend(AP_Baro &baro) : 
    _frontend(baro) 
{
    _sem = hal.util->new_semaphore();    
}

void AP_Baro_Backend::update_healthy_flag(uint8_t instance)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    if (!_sem->take_nonblocking()) {
        return;
    }

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds and values are non-zero and have changed within the last 2 seconds
    const uint32_t now = AP_HAL::millis();
    _frontend.sensors[instance].healthy =
        (now - _frontend.sensors[instance].last_update_ms < BARO_TIMEOUT_MS) &&
        (now - _frontend.sensors[instance].last_change_ms < BARO_DATA_CHANGE_TIMEOUT_MS) &&
        !is_zero(_frontend.sensors[instance].pressure);

    _sem->give();
}

void AP_Baro_Backend::backend_update(uint8_t instance)
{
    update();
    update_healthy_flag(instance);
}


/*
  copy latest data to the frontend from a backend
 */
void AP_Baro_Backend::_copy_to_frontend(uint8_t instance, float pressure, float temperature)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    uint32_t now = AP_HAL::millis();

    // check for changes in data values
    if (!is_equal(_frontend.sensors[instance].pressure, pressure) || !is_equal(_frontend.sensors[instance].temperature, temperature)) {
        _frontend.sensors[instance].last_change_ms = now;
    }

    // update readings
    _frontend.sensors[instance].pressure = pressure;
    _frontend.sensors[instance].temperature = temperature;
    _frontend.sensors[instance].last_update_ms = now;
}
