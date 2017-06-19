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
    // @Units: metres
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_AHEAD", 6, SoaringController, thermal_distance_ahead, 5.0f),

    // @Param: MIN_THML_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling
    // @Units: seconds
    // @Range: 0 32768
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: seconds
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
    // @Units: m*m/s/s
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, polar_K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude, relative to the home location
    // @Description: Don't thermal any higher than this.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 12, SoaringController, alt_max, 350.0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude, relative to the home location
    // @Description: Don't get any lower than this.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 13, SoaringController, alt_min, 50.0),

    // @Param: ALT_CUTOFF
    // @DisplayName: Maximum power altitude, relative to the home location
    // @Description: Cut off throttle at this alt.
    // @Units: meters
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // @Param: ENABLE_CH
    // @DisplayName: (Optional) RC channel that toggles the soaring controller on and off
    // @Description: Toggles the soaring controller on and off. This parameter has any effect only if SOAR_ENABLE is set to 1 and this parameter is set to a valid non-zero channel number. When set, soaring will be activated when RC input to the specified channel is greater than or equal to 1700.
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("ENABLE_CH", 15, SoaringController, soar_active_ch, 0),

    AP_GROUPEND
};

SoaringController::SoaringController(AP_AHRS &ahrs, AP_SpdHgtControl &spdHgt, const AP_Vehicle::FixedWing &parms) :
    _ahrs(ahrs),
    _spdHgt(spdHgt),
    _aparm(parms),
    _new_data(false),
    _loiter_rad(parms.loiter_radius),
    _throttle_suppressed(true)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SoaringController::get_target(Location &wp)
{
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
        _filtered_vario_reading = 0;
    }

    return _throttle_suppressed;
}

bool SoaringController::check_thermal_criteria()
{
    return (soar_active
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && _filtered_vario_reading > thermal_vspeed
            && _alt < alt_max
            && _alt > alt_min);
}

bool SoaringController::check_cruise_criteria()
{
    float thermalability = (_ekf.X[0]*expf(-powf(_loiter_rad / _ekf.X[1], 2))) - EXPECTED_THERMALLING_SINK;

    if (soar_active && (AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6) && thermalability < McCready(_alt)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Thermal weak, recommend quitting: W %f R %f th %f alt %f Mc %f\n", (double)_ekf.X[0], (double)_ekf.X[1], (double)thermalability, (double)_alt, (double)McCready(_alt));
        return true;
    } else if (soar_active && (_alt>alt_max || _alt<alt_min)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ALERT, "Out of allowable altitude range, beginning cruise. Alt = %f\n", (double)_alt);
        return true;
    }

    return false;
}

bool SoaringController::check_init_thermal_criteria()
{
    if (soar_active && (AP_HAL::micros64() - _thermal_start_time_us) < ((unsigned)min_thermal_s * 1e6)) {
        return true;
    }

    return false;

}

void SoaringController::init_thermalling()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r = powf(thermal_r, 2);
    float cov_q1 = powf(thermal_q1, 2); // State covariance
    float cov_q2 = powf(thermal_q2, 2); // State covariance
    const float init_q[4] = {cov_q1, cov_q2, cov_q2, cov_q2};
    const MatrixN<float,4> q{init_q};
    const float init_p[4] = {INITIAL_STRENGTH_COVARIANCE, INITIAL_RADIUS_COVARIANCE, INITIAL_POSITION_COVARIANCE, INITIAL_POSITION_COVARIANCE};
    const MatrixN<float,4> p{init_p};

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const float init_xr[4] = {INITIAL_THERMAL_STRENGTH,
                              INITIAL_THERMAL_RADIUS,
                              thermal_distance_ahead * cosf(_ahrs.yaw),
                              thermal_distance_ahead * sinf(_ahrs.yaw)};
    const VectorN<float,4> xr{init_xr};

    // Also reset covariance matrix p so filter is not affected by previous data
    _ekf.reset(xr, p, q, r);

    _ahrs.get_position(_prev_update_location);
    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
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

void SoaringController::get_altitude_wrt_home(float *alt)
{
    _ahrs.get_relative_position_D_home(*alt);
    *alt *= -1.0f;
}
void SoaringController::update_thermalling()
{
    struct Location current_loc;
    _ahrs.get_position(current_loc);

    if (_new_data) {
        float dx = 0;
        float dy = 0;
        float dx_w = 0;
        float dy_w = 0;
        Vector3f wind = _ahrs.wind_estimate();
        get_wind_corrected_drift(&current_loc, &wind, &dx_w, &dy_w, &dx, &dy);

#if (0)
        // Print32_t filter info for debugging
        int32_t i;
        for (i = 0; i < 4; i++) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%e ", (double)_ekf.P[i][i]);
        }
        for (i = 0; i < 4; i++) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%e ", (double)_ekf.X[i]);
        }
#endif

        // write log - save the data.
        DataFlash_Class::instance()->Log_Write("SOAR", "TimeUS,nettorate,dx,dy,x0,x1,x2,x3,lat,lng,alt,dx_w,dy_w", "QfffffffLLfff", 
                                               AP_HAL::micros64(),
                                               (double)_vario_reading,
                                               (double)dx,
                                               (double)dy,
                                               (double)_ekf.X[0],
                                               (double)_ekf.X[1],
                                               (double)_ekf.X[2],
                                               (double)_ekf.X[3],
                                               current_loc.lat,
                                               current_loc.lng,
                                               (double)_alt,
                                               (double)dx_w,
                                               (double)dy_w);

        //log_data();
        _ekf.update(_vario_reading,dx, dy);       // update the filter

        _prev_update_location = current_loc;      // save for next time
        _prev_update_time = AP_HAL::micros64();
        _new_data = false;
    }
}

void SoaringController::update_cruising()
{
    // Reserved for future tasks that need to run continuously while in FBWB or AUTO mode,
    // for example, calculation of optimal airspeed and flap angle.
}

void SoaringController::update_vario()
{
    Location current_loc;
    _ahrs.get_position(current_loc);
    get_altitude_wrt_home(&_alt);

    if (fabsf(_alt - _last_alt) > 0.0001f) { // if no change in altitude then there will be no update of ekf buffer
        // Both filtered total energy rates and unfiltered are computed for the thermal switching logic and the EKF
        float aspd = 0;
        float roll = _ahrs.roll;
        if (!_ahrs.airspeed_estimate(&aspd)) {
            aspd = _aparm.airspeed_cruise_cm / 100.0f;
        }
        _aspd_filt = ASPD_FILT * aspd + (1 - ASPD_FILT) * _aspd_filt;
        float total_E = _alt + 0.5 *_aspd_filt * _aspd_filt / GRAVITY_MSS;                                                  // Work out total energy
        float sinkrate = correct_netto_rate(0.0f, (roll + _last_roll) / 2, _aspd_filt);                                     // Compute still-air sinkrate
        _vario_reading = (total_E - _last_total_E) / ((AP_HAL::micros64() - _prev_vario_update_time) * 1e-6) + sinkrate;    // Unfiltered netto rate
        _filtered_vario_reading = TE_FILT * _vario_reading + (1 - TE_FILT) * _filtered_vario_reading;                       // Apply low pass timeconst filter for noise
        _displayed_vario_reading = TE_FILT_DISPLAYED * _vario_reading + (1 - TE_FILT_DISPLAYED) * _displayed_vario_reading;

        float dx = 0;
        float dy = 0;
        float dx_w = 0;
        float dy_w = 0;
        Vector3f wind = _ahrs.wind_estimate();
        get_wind_corrected_drift(&current_loc, &wind, &dx_w, &dy_w, &dx, &dy);

        _last_alt = _alt;                                       // Store variables
        _last_roll = roll;
        _last_aspd = aspd;
        _last_total_E = total_E;
        _prev_vario_update_time = AP_HAL::micros64();
        _new_data = true;

        DataFlash_Class::instance()->Log_Write("VAR", "TimeUS,aspd_raw,aspd_filt,alt,roll,raw,filt,wx,wy,dx,dy", "Qffffffffff",
                                               AP_HAL::micros64(),
                                               (double)aspd,
                                               (double)_aspd_filt,
                                               (double)_alt,
                                               (double)roll,
                                               (double)_vario_reading,
                                               (double)_filtered_vario_reading,
                                               (double)wind.x,
                                               (double)wind.y,
                                               (double)dx,
                                               (double)dy);
    }
}

float SoaringController::correct_netto_rate(float climb_rate, float phi, float aspd)
{
    // Remove aircraft sink rate
    float CL0;  // CL0 = 2*W/(rho*S*V^2)
    float C1;   // C1 = CD0/CL0
    float C2;   // C2 = CDi0/CL0 = B*CL0
    float netto_rate;
    float cosphi;
    CL0 = polar_K / (aspd * aspd);
    C1 = polar_CD0 / CL0;  // constant describing expected angle to overcome zero-lift drag
    C2 = polar_B * CL0;    // constant describing expected angle to overcome lift induced drag at zero bank

    cosphi = (1 - phi * phi / 2); // first two terms of mclaurin series for cos(phi)
    netto_rate = climb_rate + aspd * (C1 + C2 / (cosphi * cosphi));  // effect of aircraft drag removed

    // Remove acceleration effect - needs to be tested.
    //float temp_netto = netto_rate;
    //float dVdt = SpdHgt_Controller->get_VXdot();
    //netto_rate = netto_rate + aspd*dVdt/GRAVITY_MSS;
    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "%f %f %f %f\n",temp_netto,dVdt,netto_rate,barometer.get_altitude());
    return netto_rate;
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
    return hal.rcin->read(soar_active_ch-1) >= 1700;
}
