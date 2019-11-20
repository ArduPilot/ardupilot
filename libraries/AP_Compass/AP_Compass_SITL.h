#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AP_Math/vectorN.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>

#define MAX_SITL_COMPASSES 3

class AP_Compass_SITL : public AP_Compass_Backend {
public:
    AP_Compass_SITL();

    void read(void) override;

private:
    uint8_t _compass_instance[MAX_SITL_COMPASSES];
    uint8_t _num_compass;
    SITL::SITL *_sitl;

    // delay buffer variables
    struct readings_compass {
        uint32_t time;
        Vector3f data;
    };
    uint8_t store_index;
    uint32_t last_store_time;
    static const uint8_t buffer_length = 50;
    VectorN<readings_compass,buffer_length> buffer;

    void _timer();
    uint32_t _last_sample_time;

    void _setup_eliptical_correcion();
    
    Matrix3f _eliptical_corr;
    Vector3f _last_dia;
    Vector3f _last_odi;
};
#endif // CONFIG_HAL_BOARD
