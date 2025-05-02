#include "AP_Baro_Backend.h"

#include <stdio.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;
static constexpr float FILTER_KOEF = 0.1f; 

// constructor
AP_Baro_Backend::AP_Baro_Backend(AP_Baro &baro) : 
    _frontend(baro) 
{
    _glitch_filter.init(FILTER_KOEF, 1000.0f, 1.0f);
}

void AP_Baro_Backend::update_healthy_flag(uint8_t instance)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    WITH_SEMAPHORE(_sem);

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds and values are non-zero and have changed within the last 2 seconds
    const uint32_t now = AP_HAL::millis();
    _frontend.sensors[instance].healthy =
        (now - _frontend.sensors[instance].last_update_ms < BARO_TIMEOUT_MS) &&
        (now - _frontend.sensors[instance].last_change_ms < BARO_DATA_CHANGE_TIMEOUT_MS) &&
        !is_zero(_frontend.sensors[instance].pressure);

    if (_frontend.sensors[instance].temperature < -200 ||
        _frontend.sensors[instance].temperature > 200) {
        // if temperature is way out of range then we likely have bad
        // data from the sensor, treat is as unhealthy. This is done
        // so SPI sensors which have no data validity checking can
        // mark a sensor unhealthy
        _frontend.sensors[instance].healthy = false;
    }
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



/* Check that the baro value is valid by using a mean filter. If the
 * value is further than filter_range from mean value, it is
 * rejected. 
*/
bool AP_Baro_Backend::pressure_ok(float press)
{
   
    const float range = (float)_frontend.get_filter_range();
    
    if (range <= 0) {
        return true;
    }

    return !_glitch_filter.is_glitch(range, press);
}
