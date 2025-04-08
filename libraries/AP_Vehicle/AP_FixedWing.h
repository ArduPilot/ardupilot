#pragma once

#include <AP_Param/AP_Param.h>

/*
  common parameters for fixed wing aircraft
*/
struct AP_FixedWing {
    AP_Int8 throttle_min;
    AP_Int8 throttle_max;
    AP_Int8 throttle_slewrate;
    AP_Int8 throttle_cruise;
    AP_Int8 takeoff_throttle_max;
    AP_Int8 takeoff_throttle_min;
    AP_Int8 takeoff_throttle_idle;
    AP_Int32 takeoff_options;
    AP_Int16 airspeed_min;
    AP_Int16 airspeed_max;
    AP_Float airspeed_cruise;
    AP_Float airspeed_stall;
    AP_Float min_groundspeed;
    AP_Int8  crash_detection_enable;
    AP_Float roll_limit;
    AP_Float pitch_limit_max;
    AP_Float pitch_limit_min;
    AP_Int8  autotune_level;
    AP_Int32 autotune_options;
    AP_Int8  stall_prevention;
    AP_Int16 loiter_radius;
    AP_Float takeoff_throttle_max_t;

    struct Rangefinder_State {
        bool in_range:1;
        bool have_initial_reading:1;
        bool in_use:1;
        float initial_range;
        float correction;
        float initial_correction;
        float last_stable_correction;
        uint32_t last_correction_time_ms;
        uint8_t in_range_count;
        float height_estimate;
        float last_distance;
    };


    // stages of flight
    enum class FlightStage {
        TAKEOFF       = 1,
        VTOL          = 2,
        NORMAL        = 3,
        LAND          = 4,
        ABORT_LANDING = 7
    };

    // Bitfields of TKOFF_OPTIONS
    enum class TakeoffOption {
        THROTTLE_RANGE          = (1U << 0), // Unset: Max throttle. Set: Throttle range.
    };
};
