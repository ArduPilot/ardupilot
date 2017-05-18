/*
  Soaring Controller class by Samuel Tabor

  Provides a layer between the thermal centring algorithm and the main
  code for managing navigation targets, data logging, tuning parameters,
  algorithm inputs and eventually other soaring strategies such as
  speed-to-fly. AP_TECS libary used for reference.
*/

#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <DataFlash/DataFlash.h>
#include <AP_Math/AP_Math.h>
#include "ExtendedKalmanFilter.h"
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>

#define EXPECTED_THERMALLING_SINK 0.7
#define INITIAL_THERMAL_STRENGTH 2.0
#define INITIAL_THERMAL_RADIUS 30.0 //150.0
#define INITIAL_STRENGTH_COVARIANCE 0.0049
#define INITIAL_RADIUS_COVARIANCE 2500.0
#define INITIAL_POSITION_COVARIANCE 300.0
#define ASPD_FILT 0.05
#define TE_FILT 0.03
#define TE_FILT_DISPLAYED 0.15

class SoaringController {
    ExtendedKalmanFilter _ekf{};
    AP_AHRS &_ahrs;
    AP_SpdHgtControl &_spdHgt;
    const AP_Vehicle::FixedWing &_aparm;

    // store aircraft location at last update
    struct Location _prev_update_location;

    // store time thermal was entered for hysteresis
    unsigned long _thermal_start_time_us;

    // store time cruise was entered for hysteresis
    unsigned long _cruise_start_time_us;

    // store time of last update
    unsigned long _prev_update_time;

    // store time of last update
    unsigned long _prev_vario_update_time;

    float _vario_reading;
    float _filtered_vario_reading;
    float _last_alt;
    float _alt;
    float _last_aspd;
    float _last_roll;
    float _last_total_E;
    bool _new_data;
    float _loiter_rad;
    bool _throttle_suppressed;

    float _displayed_vario_reading;
    float _aspd_filt;
    float correct_netto_rate(float climb_rate, float phi, float aspd);
    float McCready(float alt);
    void get_wind_corrected_drift(const Location *current_loc, const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy);
    void get_altitude_wrt_home(float *alt);

protected:
    AP_Int8 soar_active;
    AP_Int8 soar_active_ch;
    AP_Float thermal_vspeed;
    AP_Float thermal_q1;
    AP_Float thermal_q2;
    AP_Float thermal_r;
    AP_Float thermal_distance_ahead;
    AP_Int16 min_thermal_s;
    AP_Int16 min_cruise_s;
    AP_Float polar_CD0;
    AP_Float polar_B;
    AP_Float polar_K;
    AP_Float alt_max;
    AP_Float alt_min;
    AP_Float alt_cutoff;

public:
    SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms);

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    void get_target(Location & wp);
    bool suppress_throttle();
    bool check_thermal_criteria();
    bool check_cruise_criteria();
    bool check_init_thermal_criteria();
    void init_thermalling();
    void init_cruising();
    void update_thermalling();
    void update_cruising();
    bool is_active() const;
    bool get_throttle_suppressed() const
    {
        return _throttle_suppressed;
    }
    void set_throttle_suppressed(bool suppressed)
    {
        _throttle_suppressed = suppressed;
    }
    float get_vario_reading()
    {
        return _displayed_vario_reading;
    }

    void update_vario();
};
