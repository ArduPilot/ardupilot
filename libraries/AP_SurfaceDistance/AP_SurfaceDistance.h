#pragma once

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_HAL/Semaphores.h>

class AP_SurfaceDistance {
public:
    AP_SurfaceDistance(Rotation rot, const AP_InertialNav& inav, uint8_t i) :
        instance(i),
        inertial_nav(inav),
        rotation(rot)
    {};

    void update();

    // check if the last healthy range finder reading is too old to be considered valid
    bool data_stale(void);

    // helper to check that rangefinder was last reported as enabled and healthy
    bool enabled_and_healthy(void) const;

    // Check that there is a rangefinder in the correct orientation configured, used for pre-arm checks
    // when the rangefinder may not be reporting healthy (e.g. on ground reporting out of range low)
    bool rangefinder_configured(void) const;

    // get inertially interpolated rangefinder height
    bool get_rangefinder_height_interpolated_cm(int32_t& ret, const uint32_t oor_low_timeout_ms = 0);

    bool enabled;                          // not to be confused with rangefinder enabled, this state is to be set by the vehicle.
    bool alt_healthy;                      // true if we can trust the altitude from the rangefinder
    int32_t alt_cm;                        // tilt compensated altitude (in cm) from rangefinder
    float inertial_alt_cm;                 // inertial alt at time of last rangefinder sample
    LowPassFilterFloat alt_cm_filt {0.5};  // altitude filter
    int32_t alt_cm_glitch_protected;       // last glitch protected altitude
    int8_t glitch_count;                   // non-zero number indicates rangefinder is glitching
    uint32_t glitch_cleared_ms;            // system time glitch cleared
    float terrain_offset_cm;               // filtered terrain offset (e.g. terrain's height above EKF origin)

private:
#if HAL_LOGGING_ENABLED
    void Log_Write(void) const;
#endif

    // multi-thread access support
    HAL_Semaphore sem;

    bool rangefinder_is_config;
    bool has_been_healthy;
    uint32_t out_of_range_low_ms;           // keep track of the rangefinder state. When using the inertially interpolated rangefinder reading sometimes it is acceptable to use the rangefinder when it is reporting low
    const uint8_t instance;
    uint8_t status;
    uint32_t last_healthy_ms;

    const AP_InertialNav& inertial_nav;
    const Rotation rotation;
};
