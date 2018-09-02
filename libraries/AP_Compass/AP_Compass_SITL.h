#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AP_Math/vectorN.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>

#define SITL_NUM_COMPASSES 2

class AP_Compass_SITL : public AP_Compass_Backend {
public:
    AP_Compass_SITL(Compass &);

    void read(void);

private:
    uint8_t _compass_instance[SITL_NUM_COMPASSES];
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
    bool _has_sample;
    uint32_t _last_sample_time;

    Vector3f _mag_accum[SITL_NUM_COMPASSES];
    uint32_t _accum_count;

    void _setup_eliptical_correcion();
    
    Matrix3f _eliptical_corr;
    Vector3f _last_dia;
    Vector3f _last_odi;
};
#endif // CONFIG_HAL_BOARD
