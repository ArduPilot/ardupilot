#pragma once

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <AP_InertialNav/AP_InertialNav.h>

class AC_SurfaceDistance {
public:
    AC_SurfaceDistance(Rotation rot, const AP_InertialNav& inav):rotation(rot),inertial_nav(inav) {};

    void update();

    bool enabled;
    bool alt_healthy; // true if we can trust the altitude from the rangefinder
    int16_t alt_cm; // tilt compensated altitude (in cm) from rangefinder
    float inertial_alt_cm; // inertial alt at time of last rangefinder sample
    uint32_t last_healthy_ms;
    LowPassFilterFloat alt_cm_filt {0.5}; // altitude filter
    int16_t alt_cm_glitch_protected;    // last glitch protected altitude
    int8_t glitch_count;    // non-zero number indicates rangefinder is glitching
    uint32_t glitch_cleared_ms; // system time glitch cleared

    const AP_InertialNav& inertial_nav;
    const Rotation rotation;
};