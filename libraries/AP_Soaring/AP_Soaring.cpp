#include "AP_Soaring.h"
#include <AP_Logger/AP_Logger.h>
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
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
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
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, polar_CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
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

    // @Param: MAX_DRIFT
    // @DisplayName: (Optional) Maximum drift distance to allow when thermalling.
    // @Description: The previous mode will be restored if the horizontal distance to the thermalling start location exceeds this value. 0 to disable.
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("MAX_DRIFT", 16, SoaringController, max_drift, 0),

    AP_GROUPEND
};

SoaringController::SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms) :
    _ahrs(ahrs),
    _spdHgt(spdHgt),
    _vario(ahrs,parms),
    _aparm(parms),
    _throttle_suppressed(true)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SoaringController::get_target(Location &wp)
{
    wp = _ahrs.get_home();
    wp.offset(_ekf.X[2], _ekf.X[3]);
}

bool SoaringController::suppress_throttle()
{
    float alt = _vario.alt;

    if (_throttle_suppressed && (alt < alt_min)) {
        // Time to throttle up
        set_throttle_suppressed(false);
    } else if ((!_throttle_suppressed) && (alt > alt_cutoff)) {
        // Start glide
        set_throttle_suppressed(true);

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


SoaringController::LoiterStatus SoaringController::check_cruise_criteria()
{
    if (!soar_active) {
        _cruise_criteria_msg_last = SOARING_DISABLED;
        return SOARING_DISABLED;
    }

    LoiterStatus result = THERMAL_GOOD_TO_KEEP_LOITERING;
    const float alt = _vario.alt;

    Vector3f position;
    if (!_ahrs.get_relative_position_NED_home(position)) {
        return SOARING_DISABLED;
    }

    if (alt > alt_max) {
        result = ALT_TOO_HIGH;
        if (result != _cruise_criteria_msg_last) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Reached upper alt = %dm", (int16_t)alt);
        }
    } else if (alt < alt_min) {
        result = ALT_TOO_LOW;
        if (result != _cruise_criteria_msg_last) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Reached lower alt = %dm", (int16_t)alt);
        }
    } else if ((AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6)) {
        const float thermalability = (_ekf.X[0]*expf(-powf(_aparm.loiter_radius / _ekf.X[1], 2))) - EXPECTED_THERMALLING_SINK;
        const float mcCreadyAlt = McCready(alt);
        if (thermalability < mcCreadyAlt) {
            result = THERMAL_WEAK;
            if (result != _cruise_criteria_msg_last) {
                gcs().send_text(MAV_SEVERITY_INFO, "Thermal weak: W %f.3 R %f.3 th %f.1 alt %dm Mc %dm", (double)_ekf.X[0], (double)_ekf.X[1], (double)thermalability, (int32_t)alt, (int32_t)mcCreadyAlt);
            }
        } else if (alt < (-_thermal_start_pos.z) || _vario.smoothed_climb_rate < 0.0) {
            result = ALT_LOST;
            if (result != _cruise_criteria_msg_last) {
                gcs().send_text(MAV_SEVERITY_INFO, "Not climbing");
            }
        } else if (!(max_drift<0) && (powf(position.x - _thermal_start_pos.x, 2) + powf(position.y - _thermal_start_pos.y, 2)) > powf(max_drift,2)) {
            result = DRIFT_EXCEEDED;
            if (result != _cruise_criteria_msg_last) {
                gcs().send_text(MAV_SEVERITY_INFO, "Drifted too far");
            }
        }
    }

    _cruise_criteria_msg_last = result;
    return result;
}

void SoaringController::init_thermalling()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r      = powf(thermal_r,  2); // Measurement noise
    float cov_q1 = powf(thermal_q1, 2); // Process noise for strength
    float cov_q2 = powf(thermal_q2, 2); // Process noise for position and radius

    const float init_q[4] = {cov_q1,
                             cov_q2,
                             cov_q2,
                             cov_q2};

    const MatrixN<float,4> q{init_q};

    const float init_p[4] = {INITIAL_STRENGTH_COVARIANCE,
                             INITIAL_RADIUS_COVARIANCE,
                             INITIAL_POSITION_COVARIANCE,
                             INITIAL_POSITION_COVARIANCE};

    const MatrixN<float,4> p{init_p};

    Vector3f position;
    
    if (!_ahrs.get_relative_position_NED_home(position)) {
        return;
    }

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const float init_xr[4] = {INITIAL_THERMAL_STRENGTH,
                              INITIAL_THERMAL_RADIUS,
                              position.x + thermal_distance_ahead * cosf(_ahrs.yaw),
                              position.y + thermal_distance_ahead * sinf(_ahrs.yaw)};

    const VectorN<float,4> xr{init_xr};

    // Also reset covariance matrix p so filter is not affected by previous data
    _ekf.reset(xr, p, q, r);

    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
    _thermal_start_pos = position;

    _vario.reset_filter(0.0);
}

void SoaringController::init_cruising()
{
    if (is_active()) {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        set_throttle_suppressed(true);
    }
}

void SoaringController::update_thermalling()
{
    float deltaT = (AP_HAL::micros64() - _prev_update_time) * 1e-6;

    Vector3f current_position;

    if (!_ahrs.get_relative_position_NED_home(current_position)) {
        return;
    }

    Vector3f wind_drift = _ahrs.wind_estimate()*deltaT;

    // write log - save the data.
    AP::logger().Write("SOAR", "TimeUS,nettorate,x0,x1,x2,x3,north,east,alt,dx_w,dy_w", "Qffffffffff",
                                           AP_HAL::micros64(),
                                           (double)_vario.reading,
                                           (double)_ekf.X[0],
                                           (double)_ekf.X[1],
                                           (double)_ekf.X[2],
                                           (double)_ekf.X[3],
                                           current_position.x,
                                           current_position.y,
                                           (double)_vario.alt,
                                           (double)wind_drift.x,
                                           (double)wind_drift.y);

    // update the filter
    _ekf.update(_vario.reading, current_position.x, current_position.y, wind_drift.x, wind_drift.y);

    _prev_update_time = AP_HAL::micros64();
}

void SoaringController::update_cruising()
{
    // Reserved for future tasks that need to run continuously while in FBWB or AUTO mode,
    // for example, calculation of optimal airspeed and flap angle.
}

void SoaringController::update_vario()
{
    _vario.update(polar_K, polar_CD0, polar_B);
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

void SoaringController::set_throttle_suppressed(bool suppressed)
{
    _throttle_suppressed = suppressed;

    // Let the TECS know.
    _spdHgt.set_gliding_requested_flag(suppressed);
}