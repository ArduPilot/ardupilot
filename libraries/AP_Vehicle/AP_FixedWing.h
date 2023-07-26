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
    AP_Int16 airspeed_min;
    AP_Int16 airspeed_max;
    AP_Int32 airspeed_cruise_cm;
    AP_Int32 min_gndspeed_cm;
    AP_Int8  crash_detection_enable;
    AP_Int16 roll_limit_cd;
    AP_Int16 pitch_limit_max_cd;
    AP_Int16 pitch_limit_min_cd;
    AP_Int8  autotune_level;
    AP_Int8  stall_prevention;
    AP_Int16 loiter_radius;
    AP_Int16 pitch_trim_cd;
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
};
