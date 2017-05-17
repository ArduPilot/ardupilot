#pragma once

#include "AP_Baro_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AP_Math/vectorN.h>

class AP_Baro_SITL : public AP_Baro_Backend {
public:
    AP_Baro_SITL(AP_Baro &);

    void update() override;

private:
    uint8_t instance;
    SITL::SITL *sitl;

    // barometer delay buffer variables
    struct readings_baro {
        uint32_t time;
        float data;
    };
    uint8_t store_index;
    uint32_t last_store_time;
    static const uint8_t buffer_length = 50;
    VectorN<readings_baro,buffer_length> buffer;

    // adjust for simulated board temperature
    void temperature_adjustment(float &p, float &T);
    
};
#endif // CONFIG_HAL_BOARD

