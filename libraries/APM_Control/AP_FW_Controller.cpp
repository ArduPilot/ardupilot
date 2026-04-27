/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//	Code by Jon Challinger
//  Modified by Paul Riseborough
//


#include "AP_FW_Controller.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

AP_FW_Controller::AP_FW_Controller(const AP_FixedWing &parms, const AC_PID::Defaults &defaults, AP_AutoTune::ATType _autotune_type)
    : aparm(parms),
      rate_pid(defaults),
      autotune_type(_autotune_type)
{
    rate_pid.set_slew_limit_scale(45);
}

// Return true if input shaping should be used
bool AP_FW_Controller::apply_input_shaping() const
{
    // Must be using rate limits
    if (!apply_rate_limits()) {
        return false;
    }

    // Accel limit and time constant must be set
    if (!is_positive(accel_limit.get()) || !is_positive(aparm.input_tc.get())) {
        return false;
    }

    // auto-tune must not be running
    if ((autotune != nullptr) && autotune->running) {
        return false;
    }

    return true;
}

// Run angle controller
float AP_FW_Controller::run_angle_control(int32_t desired_angle_cd, float scaler, bool disable_integrator, bool ground_mode)
{
    // Ensure tau is valid
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }
    const float desired_angle_deg = wrap_180(desired_angle_cd * 0.01);

    if (!apply_input_shaping()) {
        // Calculate rate directly from angle error with no input shaping
        angle_err_deg = wrap_180(desired_angle_deg - get_measured_angle());
        float desired_rate = (angle_err_deg / gains.tau) + get_ff_rate_target();

        // Apply rate limits if enabled
        if (apply_rate_limits()) {
            desired_rate = rate_limit(desired_rate);
        }

        // Reset input shaping set points
        reset_input_shaping(desired_angle_deg, desired_rate);

        // Run rate controller
        return run_axis_rate_control(desired_rate, scaler, disable_integrator, ground_mode);
    }

    // Apply input shaping to desired angle
    const float dt = AP::scheduler().get_loop_period_s();

    const float accel_max = accel_limit.get();
    const float jerk_limit = accel_max / aparm.input_tc.get();

    // Ensure the shortest path is taken
    const float angle_error = wrap_180(desired_angle_deg - angle_target_deg);

    // Apply input shaping updating the accel target
    shape_pos_vel_accel(
        angle_error, get_ff_rate_target(), 0.0, // desired pos, vel and accel
        0.0, rate_target_deg, accel_target_deg, // current shaped target
        -get_negative_rate_limit(), get_positive_rate_limit(), // velocity limits
        -accel_max, accel_max, // accel limits
        jerk_limit, // jerk limit
        dt, true
    );

    // Integrate pos and vel from updated accel target
    angle_target_deg += rate_target_deg * dt + accel_target_deg * 0.5 * sq(dt);
    rate_target_deg += accel_target_deg * dt;

    // Make sure target remains in the range +-180
    angle_target_deg = wrap_180(angle_target_deg);

    // Calculate angle error
    angle_err_deg = wrap_180(angle_target_deg - get_measured_angle());

    // Apply gain using sqrt controller
    float desired_rate = sqrt_controller(angle_err_deg, 1.0 / gains.tau.get(), accel_max * 0.5, dt);

    // Add feed forward rate demand and constrain to rate limit
    desired_rate = rate_limit(desired_rate + rate_target_deg);

    // Run rate controller
    return run_axis_rate_control(desired_rate, scaler, disable_integrator, ground_mode);
}

/*
  AC_PID based rate controller
*/
float AP_FW_Controller::run_rate_control(float desired_rate, float scaler, bool disable_integrator, bool ground_mode)
{
    const float dt = AP::scheduler().get_loop_period_s();

    const float eas2tas = AP::ahrs().get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    const float rate = get_measured_rate();
    const float old_I = rate_pid.get_i();

    const bool underspeed = is_underspeed();
    if (underspeed) {
        limit_I = true;
    }

    // the PID elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs (target and measurement)
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate * scaler * scaler, dt, limit_I);

    if (underspeed) {
        // when underspeed we lock the integrator
        rate_pid.set_integrator(old_I);
    }

    // FF and DFF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(ff_scale * rate_pid.get_ff_component() / (scaler * eas2tas));
    const float dff = degrees(ff_scale * rate_pid.get_dff_component() / (scaler * eas2tas));
    ff_scale = 1.0;

    if (disable_integrator) {
        rate_pid.reset_I();
    }

    // convert AC_PID info object to same scale as old controller
    _pid_info = rate_pid.get_pid_info();
    auto &pinfo = _pid_info;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;
    pinfo.DFF = dff;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D + pinfo.DFF;
    if (ground_mode) {
        // when on ground suppress D and half P term to prevent oscillations
        out -= pinfo.D + 0.5*pinfo.P;
    }

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running && get_airspeed() > aparm.airspeed_min) {
        // let autotune have a go at the values
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

/*
 Function returns an equivalent control surface deflection in centi-degrees in the range from -4500 to 4500
*/
float AP_FW_Controller::run_rate_control(float desired_rate, float scaler)
{
    // Zero angle error in pure rate control
    angle_err_deg = 0.0;

    if (!apply_input_shaping()) {
        // Reset input shaping set points
        reset_input_shaping(get_measured_angle(), desired_rate);

        // run rate control with no input shaping
        return run_rate_control(desired_rate, scaler, false, false);
    }

    // Apply input shaping to desired rate
    const float dt = AP::scheduler().get_loop_period_s();

    // Rest the input shaping target angle
    angle_target_deg = get_measured_angle();

    const float accel_max = accel_limit.get();
    const float tc = MAX(aparm.input_tc.get(), 0.1);
    const float jerk_limit = accel_max / tc;

    // Apply input shaping updating the accel target
    shape_pos_vel_accel(
        0.0, desired_rate, 0.0,                 // desired pos, vel and accel
        0.0, rate_target_deg, accel_target_deg, // current shaped target
        -get_negative_rate_limit(), get_positive_rate_limit(), // velocity limits
        -accel_max, accel_max, // accel limits
        jerk_limit, // jerk limit
        dt, true
    );

    rate_target_deg += accel_target_deg * dt;

    // Run rate controller
    return run_rate_control(desired_rate, scaler, false, false);
}

// Reset I term
void AP_FW_Controller::reset_I()
{
    rate_pid.reset_I();
    _last_out = 0.0;
}

/*
    reduce the integrator, used when we have a low scale factor in a quadplane hover
*/
void AP_FW_Controller::decay_I()
{
    // this reduces integrator by 95% over 2s
    _pid_info.I *= 0.995f;
    rate_pid.set_integrator(rate_pid.get_i() * 0.995);
}

/*
  restore autotune gains
 */
void AP_FW_Controller::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}

/*
  start an autotune
 */
void AP_FW_Controller::autotune_start(void)
{
    if (autotune == nullptr) {
        autotune = NEW_NOTHROW AP_AutoTune(gains, autotune_type, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed %s allocation", AP_AutoTune::axis_string(autotune_type));
            }
            failed_autotune_alloc = true;
        }
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

// Return the airspeed in m/s
float AP_FW_Controller::get_airspeed() const
{
    float aspeed;
    if (!AP::ahrs().airspeed_EAS(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    return aspeed;
}

// Reset controller
void AP_FW_Controller::reset()
{
    // Reset PID
    rate_pid.reset_I();
    rate_pid.reset_filter();
    _last_out = 0.0;

    // Reset input shaping
    reset_input_shaping(get_measured_angle(), get_measured_rate());
}

// Apply positive and negative rate limits to passed in value
float AP_FW_Controller::rate_limit(float rate) const
{
    const float pos_rate_limit = get_positive_rate_limit();
    if (is_positive(pos_rate_limit)) {
        rate = MIN(rate, pos_rate_limit);
    }

    const float neg_rate_limit = get_negative_rate_limit();
    if (is_positive(neg_rate_limit)) {
        rate = MAX(rate, -neg_rate_limit);
    }

    return rate;
}

// Reset input shaping applying rate limits
void AP_FW_Controller::reset_input_shaping(const float angle, const float rate)
{
    // No angle limits at the controller level, reset to the passed in angle
    angle_target_deg = angle;

    // Reset to passed in rate and apply rate limits
    rate_target_deg = rate_limit(rate);

    // Reset accel to zero
    accel_target_deg = 0.0;
}

// Get input shaping angle, rate and accel for logging
void AP_FW_Controller::get_input_shaping(float &angle, float &rate, float &accel) const
{
    angle = angle_target_deg;
    rate = rate_target_deg;
    accel = accel_target_deg;
}
