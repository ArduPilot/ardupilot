/*
  Soaring Controller class by Samuel Tabor

  Provides a layer between the thermal centring algorithm and the main
  code for managing navigation targets, data logging, tuning parameters,
  algorithm inputs and eventually other soaring strategies such as
  speed-to-fly. AP_TECS library used for reference.
*/

#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "ExtendedKalmanFilter.h"
#include "Variometer.h"
#include <AP_TECS/AP_TECS.h>
#include "SpeedToFly.h"

#ifndef HAL_SOARING_ENABLED
 #define HAL_SOARING_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_SOARING_ENABLED

#define INITIAL_THERMAL_RADIUS 80.0
#define INITIAL_STRENGTH_COVARIANCE 0.0049
#define INITIAL_RADIUS_COVARIANCE 400.0
#define INITIAL_POSITION_COVARIANCE 400.0


class SoaringController {
    Variometer::PolarParams _polarParams;
    ExtendedKalmanFilter _ekf{};
    AP_TECS &_tecs;
    Variometer _vario;
    SpeedToFly _speedToFly;

    const AP_Vehicle::FixedWing &_aparm;

    // store aircraft location at last update
    Vector3f _prev_update_location;

    // store time thermal was entered for hysteresis
    uint64_t _thermal_start_time_us;

    // store position thermal was entered as a backup check
    Vector3f _thermal_start_pos;

    // store time cruise was entered for hysteresis
    uint64_t _cruise_start_time_us;

    // store time of last update
    uint64_t _prev_update_time;

    bool _throttle_suppressed;

    float McCready(float alt);

    float _thermalability;

    LowPassFilter<float> _position_x_filter{1/60.0};
    LowPassFilter<float> _position_y_filter{1/60.0};

protected:
    AP_Int8 soar_active;
    AP_Float thermal_vspeed;
    AP_Float thermal_q1;
    AP_Float thermal_q2;
    AP_Float thermal_r;
    AP_Float thermal_distance_ahead;
    AP_Int16 min_thermal_s;
    AP_Int16 min_cruise_s;
    AP_Float alt_max;
    AP_Float alt_min;
    AP_Float alt_cutoff;
    AP_Float max_drift;
    AP_Float thermal_bank;
    AP_Float soar_thermal_airspeed;
    AP_Float soar_cruise_airspeed;
    AP_Float soar_thermal_flap;

public:
    SoaringController(AP_TECS &tecs, const AP_Vehicle::FixedWing &parms);

    enum class LoiterStatus {
        DISABLED,
        ALT_TOO_HIGH,
        ALT_TOO_LOW,
        THERMAL_WEAK,
        ALT_LOST,
        DRIFT_EXCEEDED,
        GOOD_TO_KEEP_LOITERING,
        EXIT_COMMANDED,
    };

    enum class ActiveStatus {
        SOARING_DISABLED,
        MANUAL_MODE_CHANGE,
        AUTO_MODE_CHANGE
    };

    AP_Float max_radius;

    // this supports the TECS_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    void get_target(Location & wp);
    bool suppress_throttle();
    bool check_thermal_criteria();
    LoiterStatus check_cruise_criteria(Vector2f prev_wp, Vector2f next_wp);
    void init_thermalling();
    void init_cruising();
    void update_thermalling();
    void update_cruising();
    void set_throttle_suppressed(bool suppressed);

    bool get_throttle_suppressed() const
    {
        return _throttle_suppressed;
    }

    float get_vario_reading() const
    {
        return _vario.get_displayed_value();
    }

    void update_vario();

    bool check_drift(Vector2f prev_wp, Vector2f next_wp);

    void update_active_state(bool override_disable);

    bool is_active() const {return _last_update_status>=SoaringController::ActiveStatus::MANUAL_MODE_CHANGE;};

    void set_pilot_desired_state(ActiveStatus pilot_desired_state) {_pilot_desired_state = pilot_desired_state;};

    float get_alt_cutoff() const {return alt_cutoff;}

    float get_circling_time() const {return _vario.tau;}

    float get_thermalling_radius() const;

    float get_thermalling_target_airspeed();

    float get_cruising_target_airspeed();

    float get_thermalling_flap() const
    {
        return soar_thermal_flap;
    }

private:

    ActiveStatus _last_update_status;

    ActiveStatus _pilot_desired_state = ActiveStatus::AUTO_MODE_CHANGE;

    ActiveStatus active_state(bool override_disable) const;

    bool _exit_commanded;
};

#endif // HAL_SOARING_ENABLED
