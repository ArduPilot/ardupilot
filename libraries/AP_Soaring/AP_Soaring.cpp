#include "AP_Soaring.h"
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>
extern const AP_HAL::HAL& hal;


// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the soaring mode enabled or not
    // @Description: Toggles the soaring mode on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, SoaringController, soar_active, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling speed
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("VSPEED", 2, SoaringController, thermal_vspeed, 0.7f),

    // @Param: Q1
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for strength
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Units:
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Units:
    // @Range: 0 10
    // @User: Advanced

    AP_GROUPINFO("R", 5, SoaringController, thermal_r, 0.45f),

    // @Param: DIST_AHEAD
    // @DisplayName: Distance to thermal center
    // @Description: Initial guess of the distance to the thermal center
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_AHEAD", 6, SoaringController, thermal_distance_ahead, 5.0f),

    // @Param: MIN_THML_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling
    // @Units: s
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: s
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_CRSE_S", 8, SoaringController, min_cruise_s, 30),

    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coef.
    // @Description: Zero lift drag coefficient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, polar_CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Units:
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_B", 10, SoaringController, polar_B, 0.031),

    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor 2*m*g/(rho*S)
    // @Units: m.m/s/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, polar_K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude, relative to the home location
    // @Description: Don't thermal any higher than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 12, SoaringController, alt_max, 350.0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude, relative to the home location
    // @Description: Don't get any lower than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 13, SoaringController, alt_min, 50.0),

    // @Param: ALT_CUTOFF
    // @DisplayName: Maximum power altitude, relative to the home location
    // @Description: Cut off throttle at this alt.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // @Param: ENABLE_CH
    // @DisplayName: (Optional) RC channel that toggles the soaring controller on and off
    // @Description: Toggles the soaring controller on and off. This parameter has any effect only if SOAR_ENABLE is set to 1 and this parameter is set to a valid non-zero channel number. When set, soaring will be activated when RC input to the specified channel is greater than or equal to 1700.
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("ENABLE_CH", 15, SoaringController, soar_active_ch, 0),

    // @Param: EXIT_MODE
    // @DisplayName: Thermal exit mode
    // @Description: Thermal exit mode. 0 = ArduSoar, 1 (recommended) or 2 = POMDP. It's possible to use ArduSoar's thermal exit mode with POMDSoar, but ArduSoar can only use its own thermal exit mode, 0.
    // @Units:
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("EXIT_MODE", 16, SoaringController, exit_mode, 0),

    //POMDP params.
    AP_SUBGROUPINFO(_pomdsoar, "POMD_", 17, SoaringController, POMDSoarAlgorithm),

    AP_GROUPEND
};


SoaringController::SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms, AP_RollController  &rollController, AP_Float &scaling_speed) :
    _ahrs(ahrs),
    _spdHgt(spdHgt),
    _vario(ahrs,parms),
    _loiter_rad(parms.loiter_radius),
    _throttle_suppressed(true),
    _pomdsoar(this, rollController, scaling_speed)
{
    AP_Param::setup_object_defaults(this, var_info);
    _prev_update_time = AP_HAL::micros64();
}

void SoaringController::get_target(Location &wp)
{
    // Set waypoint regardless of whether its used. This way estimated thermal is shown on GCS.
    wp = _prev_update_location;
    location_offset(wp, _ekf.X[2], _ekf.X[3]);
}

bool SoaringController::suppress_throttle()
{
    float alt = 0;
    get_altitude_wrt_home(&alt);

    if (_throttle_suppressed && (alt < alt_min)) {
        // Time to throttle up
        _throttle_suppressed = false;
    } else if ((!_throttle_suppressed) && (alt > alt_cutoff)) {
        // Start glide
        _throttle_suppressed = true;
        // Zero the pitch integrator - the nose is currently raised to climb, we need to go back to glide.
        _spdHgt.reset_pitch_I();
        _cruise_start_time_us = AP_HAL::micros64();
        // Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
        // leading to false positives.
        _vario.filtered_reading = 0;
    }

    return _throttle_suppressed;
}

bool SoaringController::check_thermal_criteria()
{
    return (soar_active
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && _vario.filtered_reading > thermal_vspeed
            && _vario.alt < alt_max
            && _vario.alt > alt_min);
}

bool SoaringController::check_cruise_criteria()
{
    float thermalability = -1e6;
    float alt = _vario.alt;

    if (_pomdsoar.pomdp_on && (exit_mode == 1 || exit_mode == 2)) {
        thermalability = _pomdsoar.assess_thermalability(uint8_t(exit_mode));
    } else {
        _loiter_rad = _aparm.loiter_radius;
        thermalability = (_ekf.X[0] * expf(-powf(_loiter_rad / _ekf.X[1], 2))) - EXPECTED_THERMALLING_SINK;
    }

    if (soar_active && !is_in_thermal_locking_period() && thermalability < McCready(alt) && _pomdsoar.ok_to_stop()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f", (double)_ekf.X[0], (double)_ekf.X[1], (double)thermalability, (double)alt, (double)McCready(alt));
        return true;
    } else if (soar_active && (alt>alt_max || alt<alt_min)) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Out of allowable altitude range, beginning cruise. Alt = %f", (double)alt);
        return true;
    }

    return false;
}


bool SoaringController::is_in_thermal_locking_period()
{
    return ((AP_HAL::micros64() - _thermal_start_time_us) < ((unsigned)min_thermal_s * 1e6));
}


void SoaringController::init_ekf()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r = powf(thermal_r, 2);
    float cov_q1 = powf(thermal_q1, 2); // State covariance
    float cov_q2 = powf(thermal_q2, 2); // State covariance
    const float init_q[4] = {cov_q1, cov_q2, cov_q2, cov_q2};
    const MatrixN<float,4> q{init_q};
    const float init_p[4] = {INITIAL_STRENGTH_COVARIANCE, INITIAL_RADIUS_COVARIANCE, INITIAL_POSITION_COVARIANCE, INITIAL_POSITION_COVARIANCE};
    const MatrixN<float,4> p{init_p};
    float ground_course = radians(_ahrs.get_gps().ground_course());
    float head_sin = sinf(ground_course); //sinf(_ahrs.yaw);
    float head_cos = cosf(ground_course); //cosf(_ahrs.yaw);

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const float init_xr[4] = {INITIAL_THERMAL_STRENGTH,
                              INITIAL_THERMAL_RADIUS,
                              thermal_distance_ahead * head_cos,
                              thermal_distance_ahead * head_sin };
    const VectorN<float,4> xr{init_xr};

    // Also reset covariance matrix p so filter is not affected by previous data
    _ekf.reset(xr, p, q, r);
}


void SoaringController::init_thermalling()
{
    _ahrs.get_position(_prev_update_location);
    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();

    init_ekf();

    if (_pomdsoar.pomdp_on) {
        _pomdsoar.init_thermalling();
    }
}

void SoaringController::init_cruising()
{
    if (is_active() && suppress_throttle()) {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        _throttle_suppressed = true;
    }
}

void SoaringController::get_wind_corrected_drift(const Location *current_loc, const Vector3f *wind, float *wind_drift_x, float *wind_drift_y, float *dx, float *dy)
{
    Vector2f diff = location_diff(_prev_update_location, *current_loc); // get distances from previous update
    *dx = diff.x;
    *dy = diff.y;

    // Wind correction
    *wind_drift_x = wind->x * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
    *wind_drift_y = wind->y * (AP_HAL::micros64() - _prev_update_time) * 1e-6;
    *dx -= *wind_drift_x;
    *dy -= *wind_drift_y;
}

void SoaringController::get_altitude_wrt_home(float *alt) const
{
    _ahrs.get_relative_position_D_home(*alt);
    *alt *= -1.0f;
}

bool SoaringController::is_controlling_roll()
{
    // Only the POMDP algorithm sets target roll directly.
    // The other method uses waypoints and the built-in navigation controller.
    return _pomdsoar.pomdp_on;
}

void SoaringController::update_thermalling()
{
    struct Location current_loc;
    _ahrs.get_position(current_loc);

    if (soar_active
        && _pomdsoar.pomdp_on
        && _pomdsoar.are_computations_in_progress()
        && (is_in_thermal_locking_period() || _pomdsoar.is_set_to_continue_past_thermal_locking_period()))
    {
        _pomdsoar.update_thermalling(current_loc);

        gcs().send_text(MAV_SEVERITY_INFO, "Action %f", _pomdsoar.get_action());

    }

    if (_vario.new_data) {
        float dx = 0;
        float dy = 0;
        float dx_w = 0;
        float dy_w = 0;
        Vector3f wind = _ahrs.wind_estimate();
        get_wind_corrected_drift(&current_loc, &wind, &dx_w, &dy_w, &dx, &dy);

        // write log - save the data.
        DataFlash_Class::instance()->Log_Write("SOAR", "TimeUS,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt,dx_w,dy_w", "QfffffffLLfff", 
                                               AP_HAL::micros64(),
                                               (double)_vario.reading,
                                               (double)dx,
                                               (double)dy,
                                               (double)_ekf.X[0],
                                               (double)_ekf.X[1],
                                               (double)_ekf.X[2],
                                               (double)_ekf.X[3],
                                               current_loc.lat,
                                               current_loc.lng,
                                               (double)_vario.alt,
                                               (double)dx_w,
                                               (double)dy_w);

        //log_data();
        _ekf.update(_vario.reading,dx, dy);       // update the filter

        _prev_update_location = current_loc;      // save for next time
        _prev_update_time = AP_HAL::micros64();
        _vario.new_data = false;
    }
}

void SoaringController::update_cruising()
{
    // Reserved for future tasks that need to run continuously while in FBWB or AUTO mode,
    // for example, calculation of optimal airspeed and flap angle.
}


void SoaringController::get_heading_estimate(float *hdx, float *hdy) const
{
    Vector2f gnd_vel = _ahrs.groundspeed_vector();
    Vector3f wind = _ahrs.wind_estimate();
    *hdx = gnd_vel.x - wind.x;
    *hdy = gnd_vel.y - wind.y;
}


void SoaringController::get_velocity_estimate(float dt, float *v0) const
{
    float hdx, hdy;
    get_heading_estimate(&hdx, &hdy);
    *v0 = sqrtf(hdx * hdx + hdy * hdy);
}


void SoaringController::update_vario()
{
    _vario.update(polar_K, polar_CD0, polar_B);
}


float SoaringController::correct_netto_rate(float climb_rate, float phi, float aspd) const
{
    return _vario.correct_netto_rate(climb_rate, phi, aspd, polar_K, polar_CD0, polar_B);
}

float SoaringController::McCready(float alt)
{
    // A method shell to be filled in later
    return thermal_vspeed;
}

bool SoaringController::is_active() const
{
    if (!soar_active) {
        return false;
    }
    if (soar_active_ch <= 0) {
        // no activation channel
        return true;
    }
    // active when above 1700
    return RC_Channels::get_radio_in(soar_active_ch-1) >= 1700;
}

void SoaringController::soaring_policy_computation()
{
    if (_pomdsoar.pomdp_on)
    {
        _pomdsoar.update_solver();
    }
}


void SoaringController::stop_computation()
{
    _pomdsoar.stop_computations();
}


float SoaringController::get_roll_cmd()
{
    return _pomdsoar.get_action();
}

float SoaringController::get_roll() const
{
    return _ahrs.roll;
}


float SoaringController::get_rate() const
{
    return _ahrs.get_gyro().x;
}


void SoaringController::get_position(Location& loc)
{
     _ahrs.get_position(loc);
}


float SoaringController::get_aspd() const
{
    // initialize to an obviously invalid value, which should get overwritten.
    float aspd = -100.0f;

    if (!_ahrs.airspeed_estimate(&aspd))
    {
        aspd = 0.5f*(_aparm.airspeed_min + _aparm.airspeed_max);
    }

    return aspd;
}


void SoaringController::get_relative_position_wrt_home(Vector2f &vec) const
{
    _ahrs.get_relative_position_NE_home(vec);
}


float SoaringController::get_eas2tas() const
{
    return _ahrs.get_EAS2TAS();
}