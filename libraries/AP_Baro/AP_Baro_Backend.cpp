#include "AP_Baro_Backend.h"

#if HAL_SENSORHUB_ENABLED
#include <AP_SensorHub/AP_SensorHub.h>
#endif

extern const AP_HAL::HAL& hal;

// constructor
AP_Baro_Backend::AP_Baro_Backend(AP_Baro &baro) :
    _frontend(baro)
{
    _sem = hal.util->new_semaphore();

#if HAL_SENSORHUB_ENABLED
    _shub = AP_SensorHub::get_instance();
#endif
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

#if HAL_SENSORHUB_ENABLED
    if (_shub && _shub->isSource()) {
        BaroMessage msg;
        msg.setInstance(instance);
        msg.setPressure(pressure);
        msg.setTemperature(temperature);
        auto packet = msg.encode();
        _shub->write(packet);
    }
#endif
}
