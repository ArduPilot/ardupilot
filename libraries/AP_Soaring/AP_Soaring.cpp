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
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Range: 0.01 1
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Range: 0.01 1
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
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: s
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("MIN_CRSE_S", 8, SoaringController, min_cruise_s, 30),

    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coef.
    // @Description: Zero lift drag coefficient
    // @Range: 0.005 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, polar_CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Range: 0.005 0.05
    // @User: Advanced
    AP_GROUPINFO("POLAR_B", 10, SoaringController, polar_B, 0.031),

    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor 2*m*g/(rho*S)
    // @Units: m.m/s/s
    // @Range: 20 400
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, polar_K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude, relative to the home location
    // @Description: Don't thermal any higher than this.
    // @Units: m
    // @Range: 0 5000.0
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
    // @Range: 0 5000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // 15 was SOAR_ENABLE_CH, now RCX_OPTION

    // @Param: MAX_DRIFT
    // @DisplayName: (Optional) Maximum drift distance to allow when thermalling.
    // @Description: The previous mode will be restored if the horizontal distance to the thermalling start location exceeds this value. -1 to disable.
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("MAX_DRIFT", 16, SoaringController, max_drift, -1),

    // @Param: MAX_RADIUS
    // @DisplayName: (Optional) Maximum distance from home
    // @Description: RTL will be entered when a thermal is exited and the plane is more than this distance from home. -1 to disable.
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("MAX_RADIUS", 17, SoaringController, max_radius, -1),

    AP_GROUPEND
};

SoaringController::SoaringController(AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms) :
    _spdHgt(spdHgt),
    _vario(parms),
    _aparm(parms),
    _throttle_suppressed(true)
{
    AP_Param::setup_object_defaults(this, var_info);

    _position_x_filter = LowPassFilter<float>(1.0/60.0);
    _position_y_filter = LowPassFilter<float>(1.0/60.0);
}

void SoaringController::get_target(Location &wp)
{
    wp = AP::ahrs().get_home();
    wp.offset(_position_x_filter.get(), _position_y_filter.get());
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
    ActiveStatus status = active_state();

    return (status == ActiveStatus::AUTO_MODE_CHANGE
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && (_vario.filtered_reading - _vario.get_exp_thermalling_sink()) > thermal_vspeed
            && _vario.alt < alt_max
            && _vario.alt > alt_min);
}


SoaringController::LoiterStatus SoaringController::check_cruise_criteria(Vector2f prev_wp, Vector2f next_wp)
{
    ActiveStatus status = active_state();

    if (status != ActiveStatus::AUTO_MODE_CHANGE) {
        _cruise_criteria_msg_last = LoiterStatus::DISABLED;
        return LoiterStatus::DISABLED;
    }

    LoiterStatus result = LoiterStatus::GOOD_TO_KEEP_LOITERING;
    const float alt = _vario.alt;

    if (alt > alt_max) {
        result = LoiterStatus::ALT_TOO_HIGH;
        if (result != _cruise_criteria_msg_last) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Reached upper alt = %dm", (int16_t)alt);
        }
    } else if (alt < alt_min) {
        result = LoiterStatus::ALT_TOO_LOW;
        if (result != _cruise_criteria_msg_last) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Reached lower alt = %dm", (int16_t)alt);
        }
    } else if ((AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6)) {
        const float mcCreadyAlt = McCready(alt);
        if (_thermalability < mcCreadyAlt) {
            result = LoiterStatus::THERMAL_WEAK;
            if (result != _cruise_criteria_msg_last) {
                gcs().send_text(MAV_SEVERITY_INFO, "Thermal weak: th %3.1fm/s alt %3.1fm Mc %3.1fm/s", (double)_thermalability, (double)alt, (double)mcCreadyAlt);
            }
        } else if (alt < (-_thermal_start_pos.z) || _vario.smoothed_climb_rate < 0.0) {
            result = LoiterStatus::ALT_LOST;
            if (result != _cruise_criteria_msg_last) {
                gcs().send_text(MAV_SEVERITY_INFO, "Not climbing");
            }
        } else if (check_drift(prev_wp, next_wp)) {
            result = LoiterStatus::DRIFT_EXCEEDED;
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

    const AP_AHRS &_ahrs = AP::ahrs();
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

    _position_x_filter.reset(_ekf.X[2]);
    _position_y_filter.reset(_ekf.X[3]);
}

void SoaringController::init_cruising()
{
    if (active_state()>=ActiveStatus::MANUAL_MODE_CHANGE) {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        set_throttle_suppressed(true);
    }
}

void SoaringController::update_thermalling()
{
    float deltaT = (AP_HAL::micros64() - _prev_update_time) * 1e-6;

    Vector3f current_position;

    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_relative_position_NED_home(current_position)) {
        return;
    }

    Vector3f wind_drift = _ahrs.wind_estimate()*deltaT*_vario.smoothed_climb_rate/_ekf.X[0];

    // update the filter
    _ekf.update(_vario.reading, current_position.x, current_position.y, wind_drift.x, wind_drift.y);

    
    _thermalability = (_ekf.X[0]*expf(-powf(_aparm.loiter_radius / _ekf.X[1], 2))) - _vario.get_exp_thermalling_sink();

    _prev_update_time = AP_HAL::micros64();

    // Compute smoothed estimate of position
    _position_x_filter.set_cutoff_frequency(1/(3*_vario.tau));
    _position_y_filter.set_cutoff_frequency(1/(3*_vario.tau));

    _position_x_filter.apply(_ekf.X[2], deltaT);
    _position_y_filter.apply(_ekf.X[3], deltaT);

    // write log - save the data.
    // @LoggerMessage: SOAR
    // @Vehicles: Plane
    // @Description: Logged data from soaring feature
    // @URL: https://ardupilot.org/plane/docs/soaring.html
    // @Field: TimeUS: microseconds since system startup
    // @Field: nettorate: Estimate of vertical speed of surrounding airmass
    // @Field: x0: Thermal strength estimate
    // @Field: x1: Thermal radius estimate
    // @Field: x2: Thermal position estimate north from home
    // @Field: x3: Thermal position estimate east from home
    // @Field: north: Aircraft position north from home
    // @Field: east: Aircraft position east from home
    // @Field: alt: Aircraft altitude
    // @Field: dx_w: Wind speed north
    // @Field: dy_w: Wind speed east
    // @Field: th: Estimate of achievable climbrate in thermal
    AP::logger().Write("SOAR", "TimeUS,nettorate,x0,x1,x2,x3,north,east,alt,dx_w,dy_w,th", "Qfffffffffff",
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
                                           (double)wind_drift.y,
                                           (double)_thermalability);
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

SoaringController::ActiveStatus SoaringController::active_state() const
{
    if (!soar_active) {
        return ActiveStatus::SOARING_DISABLED;
    }

    return _pilot_desired_state;
}

void SoaringController::update_active_state()
{
    ActiveStatus status = active_state();
    bool state_changed = !(status == _last_update_status);

    if (state_changed) {
        switch (status) {
            case ActiveStatus::SOARING_DISABLED:
                // It's not enabled, but was enabled on the last loop.
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Disabled.");
                set_throttle_suppressed(false);
                break;
            case ActiveStatus::MANUAL_MODE_CHANGE:
                // It's enabled, but wasn't on the last loop.
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Enabled, manual mode changes.");
                set_throttle_suppressed(true);
                break;
            case ActiveStatus::AUTO_MODE_CHANGE:
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Enabled, automatic mode changes.");
                set_throttle_suppressed(true);
                break;
        }
    }

    _last_update_status = status;
}


void SoaringController::set_throttle_suppressed(bool suppressed)
{
    _throttle_suppressed = suppressed;

    // Let the TECS know.
    _spdHgt.set_gliding_requested_flag(suppressed);
}

bool SoaringController::check_drift(Vector2f prev_wp, Vector2f next_wp)
{
    // Check for -1 (disabled)
    if (max_drift<0) {
        return false;
    }

    // Check against the estimated thermal.
    Vector2f position(_ekf.X[2], _ekf.X[3]);

    Vector2f start_pos(_thermal_start_pos.x, _thermal_start_pos.y);

    Vector2f mission_leg = next_wp - prev_wp;

    if (prev_wp.is_zero() || mission_leg.length() < 0.1) {
        // Simple check of distance from initial start point.
        return (position - start_pos).length() > max_drift;
    } else {
        // Regard the effective start point as projected onto mission leg.
        // Calculate drift parallel and perpendicular to mission leg.
        // Drift parallel and in direction of mission leg is acceptable.
        Vector2f effective_start, vec1, vec2;

        // Calculate effective start point (on mission leg).
        vec1 = (start_pos - prev_wp).projected(mission_leg);
        effective_start = prev_wp + vec1;

        // Calculate parallel and perpendicular offsets.
        vec2 = position - effective_start;

        float parallel      = vec2 * mission_leg.normalized();
        float perpendicular = (vec2 - mission_leg.normalized()*parallel).length();

       // Check if we've drifted beyond the next wp.
        if (parallel>(next_wp - effective_start).length()) {
            return true;
        }

        // Check if we've drifted too far laterally or backwards. We don't count positive parallel offsets
        // as these are favourable (towards next wp)
        parallel = parallel>0 ? 0 : parallel;

        return (powf(parallel,2)+powf(perpendicular,2)) > powf(max_drift,2);;
    }
}
