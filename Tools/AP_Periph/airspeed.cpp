#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_AIRSPEED

/*
  airspeed support
 */

#include <dronecan_msgs.h>

#ifndef AP_PERIPH_PROBE_CONTINUOUS
#define AP_PERIPH_PROBE_CONTINUOUS 0
#endif

/*
  update CAN airspeed
 */
void AP_Periph_FW::can_airspeed_update(void)
{
    if (!airspeed.enabled()) {
        return;
    }
#if AP_PERIPH_PROBE_CONTINUOUS
    if (!airspeed.healthy()) {
        uint32_t now = AP_HAL::millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            airspeed.allocate();
        }
    }
#endif
    uint32_t now = AP_HAL::millis();
    if (now - last_airspeed_update_ms < 50) {
        // max 20Hz data
        return;
    }
    last_airspeed_update_ms = now;
    airspeed.update();
    if (!airspeed.healthy()) {
        // don't send any data
        return;
    }
    const float press = airspeed.get_corrected_pressure();
    float temp;
    if (!airspeed.get_temperature(temp)) {
        temp = nanf("");
    } else {
        temp = C_TO_KELVIN(temp);
    }

    uavcan_equipment_air_data_RawAirData pkt {};

    // unfilled elements are NaN
    pkt.static_pressure = nanf("");
    pkt.static_pressure_sensor_temperature = nanf("");
    pkt.differential_pressure_sensor_temperature = nanf("");
    pkt.pitot_temperature = nanf("");

    // populate the elements we have
    pkt.differential_pressure = press;
    pkt.static_air_temperature = temp;

    // if a Pitot tube temperature sensor is available, use it
#if AP_TEMPERATURE_SENSOR_ENABLED
    for (uint8_t i=0; i<temperature_sensor.num_instances(); i++) {
        float temp_pitot;
        if (temperature_sensor.get_source(i) == AP_TemperatureSensor_Params::Source::Pitot_tube && temperature_sensor.get_temperature(temp_pitot, i)) {
            pkt.pitot_temperature = C_TO_KELVIN(temp_pitot);
            break;
        }
    }
#endif

    dronecan->raw_air_data_pub.broadcast(pkt);
}

#endif // HAL_PERIPH_ENABLE_AIRSPEED
