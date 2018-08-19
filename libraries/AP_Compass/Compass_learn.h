#pragma once

#include <AP_AHRS/AP_AHRS.h>

/*
  compass learning using magnetic field tables from AP_Declination
 */

class CompassLearn {
public:
    CompassLearn(AP_AHRS &ahrs, Compass &compass);

    // called on each compass read
    void update(void);

private:
    // reference to AHRS library. Needed for attitude, position and compass
    const AP_AHRS &ahrs;
    Compass &compass;
    bool have_earth_field;

    // 5 degree resolution
    static const uint16_t num_sectors = 72; 
    
    Vector3f predicted_offsets[num_sectors];
    float errors[num_sectors];
    uint32_t num_samples;

    // earth field
    Vector3f mag_ef;

    // semaphore for access to shared data with IO thread
    AP_HAL::Semaphore *sem;    
    
    struct sample {
        // milliGauss body field and offsets
        Vector3f field;
        Vector3f offsets;

        // euler radians attitude
        Vector3f attitude;
    };

    Matrix3f mat;
    
    struct sample new_sample;
    bool sample_available;
    Vector3f last_field;
    static const uint32_t min_field_change = 60;

    Vector3f best_offsets;
    float best_error;
    float best_yaw_deg;
    float worst_error;
    bool converged;

    void io_timer(void);
    void process_sample(const struct sample &s);
};
