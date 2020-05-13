
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <Filter/AverageFilter.h>
#include "EKF_Polar.h"

#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

static constexpr const uint32_t LEARN_THRESHOLD_TIME = 5000;
static constexpr const float    LEARN_THRESHOLD_ROLL = 0.2f;

class Variometer {

    AP_AHRS &_ahrs;
    const AP_Vehicle::FixedWing &_aparm;

    // store time of last update
    uint64_t _prev_update_time;

    float _last_alt;

    float _aspd_filt;
    float _aspd_filt_constrained;

    float _last_aspd;
    float _last_roll;
    float _last_total_E;
    float _expected_thermalling_sink;

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    AverageFilterFloat_Size5 _sp_filter;

    // low pass filter @ 30s time constant
    LowPassFilter<float> _climb_filter;

    LowPassFilter<float> _vdot_filter2;

    EKF_Polar _learn_EKF;
    bool _learn_initialised = false;
    uint32_t _learn_skipped_time   = 0;

public:


    struct PolarParams {
        AP_Float K;
        AP_Float CD0;
        AP_Float B;
    };

    Variometer(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, PolarParams &polarParams);
    float alt;
    float reading;
    float filtered_reading;
    float displayed_reading;
    float raw_climb_rate;
    float smoothed_climb_rate;
    float tau;


    void update();
    float calculate_aircraft_sinkrate(float phi);

    void reset_filter(float value) { _climb_filter.reset(value);}

    float get_airspeed(void) {return _aspd_filt;};

    float get_exp_thermalling_sink(void) {return _expected_thermalling_sink;};

    float calculate_circling_time_constant();

    void update_polar_learning(bool learn_enabled, bool throttle_suppressed, float dsp_dem);

private:
    PolarParams &_polarParams;

    void reset_polar_learning(void);

};

