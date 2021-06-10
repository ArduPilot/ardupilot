
/* Variometer class by Samuel Tabor

Manages the estimation of aircraft total energy, drag and vertical air velocity.
*/
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <Filter/AverageFilter.h>

#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

class Variometer {

    const AP_Vehicle::FixedWing &_aparm;

    // store time of last update
    uint64_t _prev_update_time;

    float _aspd_filt;
    float _aspd_filt_constrained;

    float _expected_thermalling_sink;

    // declares a 5point average filter using floats
    AverageFilterFloat_Size5 _vdot_filter;

    AverageFilterFloat_Size5 _sp_filter;

    // low pass filter @ 30s time constant
    LowPassFilter<float> _climb_filter{1/60.0};
    LowPassFilter<float> _vdot_filter2{1/60.0};

public:
    Variometer(const AP_Vehicle::FixedWing &parms);
    float alt;
    float reading;
    float filtered_reading;
    float displayed_reading;
    float raw_climb_rate;
    float smoothed_climb_rate;
    float tau;

    void update(const float thermal_bank, const float polar_K, const float polar_CD0, const float polar_B);
    float calculate_aircraft_sinkrate(float phi, const float polar_K, const float polar_CD0, const float polar_B) const;

    void reset_filter(float value) { _climb_filter.reset(value);}

    float get_airspeed(void) const {return _aspd_filt;};

    float get_exp_thermalling_sink(void) const {return _expected_thermalling_sink;};

    float calculate_circling_time_constant(const float thermal_bank);
};

