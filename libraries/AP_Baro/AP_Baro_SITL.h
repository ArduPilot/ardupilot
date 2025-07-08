#pragma once

#include "AP_Baro_Backend.h"

#if AP_SIM_BARO_ENABLED

#include <AP_Math/vectorN.h>

#include <SITL/SITL.h>

class AP_Baro_SITL : public AP_Baro_Backend {
public:
    AP_Baro_SITL(AP_Baro &);

    void update() override;

    // adjust for simulated board temperature
    static void temperature_adjustment(float &p, float &T);

    // adjust for wind effects
    static float wind_pressure_correction(uint8_t instance);

protected:

    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = healthy(instance); };

private:
    uint8_t _instance;
    SITL::SIM *_sitl;

    // barometer delay buffer variables
    struct readings_baro {
        uint32_t time;
        float data;
    };
    uint8_t _store_index;
    uint32_t _last_store_time;
    static const uint8_t _buffer_length = 50;
    VectorN<readings_baro, _buffer_length> _buffer;

    // is the barometer usable for flight 
    bool healthy(uint8_t instance);
    
    void _timer();
    bool _has_sample;
    uint32_t _last_sample_time;
    float _recent_temp;
    float _recent_press;
    float _last_altitude;

    uint32_t last_drift_delta_t_ms;  // allows for integration of drift over time
    float total_alt_drift;  // integrated altitude drift in metres
};
#endif  // AP_SIM_BARO_ENABLED
