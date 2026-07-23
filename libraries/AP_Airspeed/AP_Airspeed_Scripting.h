/*
  airspeed backend for airspeed data supplied by a lua script, either
  as a differential pressure or as a direct airspeed reading
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SCRIPTING_ENABLED

#include "AP_Airspeed_Backend.h"

// data timeout
#define AP_AIRSPEED_SCRIPTING_TIMEOUT_MS 500

class AP_Airspeed_Scripting : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    bool init(void) override {
        return true;
    }

    // handle a differential pressure reading, in Pascal, from a lua script
    bool handle_script_differential_pressure(float press_pa) override;

    // handle an airspeed reading, in m/s, from a lua script
    bool handle_script_airspeed(float airspeed_ms) override;

    // handle a temperature reading, in degrees C, from a lua script
    bool handle_script_temperature(float temperature_c) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

    // true once the script has ever provided an airspeed reading directly. Latches on
    // permanently so that we never fall back to differential pressure once a script
    // has chosen to provide airspeed directly
    bool has_airspeed() override;

    // return airspeed in m/s
    bool get_airspeed(float &airspeed) override;

private:
    float last_pressure;
    uint32_t last_pressure_ms;

    float last_airspeed;
    uint32_t last_airspeed_ms;
    bool have_received_airspeed;

    float last_temperature;
    uint32_t last_temperature_ms;
};

#endif  // AP_AIRSPEED_SCRIPTING_ENABLED
