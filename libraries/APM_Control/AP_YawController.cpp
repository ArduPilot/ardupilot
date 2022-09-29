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
//  Modified by Paul Riseborough to implement a three loop autopilot
//  topology
//
#include <AP_HAL/AP_HAL.h>
#include "AP_YawController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_YawController::var_info[] = {

    // @Param: 2SRV_SLIP
    // @DisplayName: Sideslip control gain
    // @Description: Gain from lateral acceleration to demanded yaw rate for aircraft with enough fuselage area to detect lateral acceleration and sideslips. Do not enable for flying wings and gliders. Actively coordinates flight more than just yaw damping. Set after YAW2SRV_DAMP and YAW2SRV_INT are tuned.
    // @Range: 0 4
    // @Increment: 0.25
    // @User: Advanced
    AP_GROUPINFO("2SRV_SLIP",    0, AP_YawController, _K_A,    0),

    // @Param: 2SRV_INT
    // @DisplayName: Sideslip control integrator
    // @Description: Integral gain from lateral acceleration error. Effectively trims rudder to eliminate long-term sideslip.
    // @Range: 0 2
    // @Increment: 0.25
    // @User: Advanced
    AP_GROUPINFO("2SRV_INT",    1, AP_YawController, _K_I,    0),

    // @Param: 2SRV_DAMP
    // @DisplayName: Yaw damping
    // @Description: Gain from yaw rate to rudder. Most effective at yaw damping and should be tuned after KFF_RDDRMIX. Also disables YAW2SRV_INT if set to 0.
    // @Range: 0 2
    // @Increment: 0.25
    // @User: Advanced
    AP_GROUPINFO("2SRV_DAMP",   2, AP_YawController, _K_D,    0),

    // @Param: 2SRV_RLL
    // @DisplayName: Yaw coordination gain
    // @Description: Gain to the yaw rate required to keep it consistent with the turn rate in a coordinated turn. Corrects for yaw tendencies after the turn is established. Increase yaw into the turn by raising. Increase yaw out of the turn by decreasing. Values outside of 0.9-1.1 range indicate airspeed calibration problems.
    // @Range: 0.8 1.2
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("2SRV_RLL",   3, AP_YawController, _K_FF,   1),

    /*
      Note: index 4 should not be used - it was used for an incorrect
      AP_Int8 version of the IMAX in the 2.74 release
     */

    // @Param: 2SRV_IMAX
    // @DisplayName: Integrator limit
    // @Description: Limit of yaw integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 1500 allows trim of up to 1/3 of servo travel range.
    // @Range: 0 4500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_IMAX",  5, AP_YawController, _imax,        1500),

    // @Param: _RATE_ENABLE
    // @DisplayName: Yaw rate enable
    // @Description: Enable yaw rate controller for aerobatic flight
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_RATE_ENABLE",  6, AP_YawController, _rate_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _RATE_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_YawController, AC_PID),

    AP_GROUPEND
};

AP_YawController::AP_YawController(const AP_FixedWing &parms)
    : aparm(parms)
{
    AP_Param::setup_object_defaults(this, var_info);
    _pid_info.target = 0;
    _pid_info.FF = 0;
    _pid_info.P = 0;
    rate_pid.set_slew_limit_scale(45);
}

int32_t AP_YawController::get_servo_out(float scaler, bool disable_integrator)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    if (_last_t == 0 || dt > 1000) {
        dt = 0;
        _pid_info.I = 0;
    }
    _last_t = tnow;


    int16_t aspd_min = aparm.airspeed_min;
    if (aspd_min < 1) {
        aspd_min = 1;
    }

    float delta_time = (float) dt * 0.001f;

    // Calculate yaw rate required to keep up with a constant height coordinated turn
    float aspeed;
    float rate_offset;
    float bank_angle = AP::ahrs().roll;
    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < 1.5707964f)	{
        bank_angle = constrain_float(bank_angle,-1.3962634f,1.3962634f);
    }
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.airspeed_estimate(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aspd_min) + float(aparm.airspeed_max));
    }
    rate_offset = (GRAVITY_MSS / MAX(aspeed, float(aspd_min))) * sinf(bank_angle) * _K_FF;

    // Get body rate vector (radians/sec)
    float omega_z = _ahrs.get_gyro().z;

    // Get the accln vector (m/s^2)
    float accel_y = AP::ins().get_accel().y;

    // subtract current bias estimate from EKF
    const Vector3f &abias = _ahrs.get_accel_bias();
    accel_y -= abias.y;

    // Subtract the steady turn component of rate from the measured rate
    // to calculate the rate relative to the turn requirement in degrees/sec
    float rate_hp_in = ToDeg(omega_z - rate_offset);

    // Apply a high-pass filter to the rate to washout any steady state error
    // due to bias errors in rate_offset
    // Use a cut-off frequency of omega = 0.2 rad/sec
    // Could make this adjustable by replacing 0.9960080 with (1 - omega * dt)
    float rate_hp_out = 0.9960080f * _last_rate_hp_out + rate_hp_in - _last_rate_hp_in;
    _last_rate_hp_out = rate_hp_out;
    _last_rate_hp_in = rate_hp_in;

    //Calculate input to integrator
    float integ_in = - _K_I * (_K_A * accel_y + rate_hp_out);

    // Apply integrator, but clamp input to prevent control saturation and freeze integrator below min FBW speed
    // Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
    // Don't integrate if _K_D is zero as integrator will keep winding up
    if (!disable_integrator && _K_D > 0) {
        //only integrate if airspeed above min value
        if (aspeed > float(aspd_min)) {
            // prevent the integrator from increasing if surface defln demand is above the upper limit
            if (_last_out < -45) {
                _integrator += MAX(integ_in * delta_time, 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                _integrator += MIN(integ_in * delta_time, 0);
            } else {
                _integrator += integ_in * delta_time;
            }
        }
    } else {
        _integrator = 0;
    }

    if (_K_D < 0.0001f) {
        // yaw damping is disabled, and the integrator is scaled by damping, so return 0
        return 0;
    }

    // Scale the integration limit
    float intLimScaled = _imax * 0.01f / (_K_D * scaler * scaler);

    // Constrain the integrator state
    _integrator = constrain_float(_integrator, -intLimScaled, intLimScaled);

    // Protect against increases to _K_D during in-flight tuning from creating large control transients
    // due to stored integrator values
    if (_K_D > _K_D_last && _K_D > 0) {
        _integrator = _K_D_last/_K_D * _integrator;
    }
    _K_D_last = _K_D;

    // Calculate demanded rudder deflection, +Ve deflection yaws nose right
    // Save to last value before application of limiter so that integrator limiting
    // can detect exceedance next frame
    // Scale using inverse dynamic pressure (1/V^2)
    _pid_info.I = _K_D * _integrator * scaler * scaler;
    _pid_info.D = _K_D * (-rate_hp_out) * scaler * scaler;
    _last_out =  _pid_info.I + _pid_info.D;

    // Convert to centi-degrees and constrain
    return constrain_float(_last_out * 100, -4500, 4500);
}

// get actuator output for direct rate control
// desired_rate is in deg/sec. scaler is the surface speed scaler
float AP_YawController::get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
    const AP_AHRS &_ahrs = AP::ahrs();

    const float dt = AP::scheduler().get_loop_period_s();
    const float eas2tas = _ahrs.get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45 || disable_integrator;
    float rate_z = _ahrs.get_gyro().z;
    float aspeed;
    float old_I = rate_pid.get_i();

    rate_pid.set_dt(dt);

    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= float(aparm.airspeed_min);
    if (underspeed) {
        limit_I = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate_z * scaler * scaler, limit_I);

    if (underspeed) {
        // when underspeed we lock the integrator
        rate_pid.set_integrator(old_I);
    }

    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(rate_pid.get_ff() / (scaler * eas2tas));

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
    pinfo.limit = limit_I;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate_z);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running && aspeed > aparm.airspeed_min) {
        // fake up an angular error based on a notional time constant of 0.5s
        const float angle_err_deg = desired_rate * gains.tau;
        // let autotune have a go at the values
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

void AP_YawController::reset_I()
{
    _pid_info.I = 0;
    rate_pid.reset_I();
    _integrator = 0;
}

void AP_YawController::reset_rate_PID()
{
    rate_pid.reset_I();
    rate_pid.reset_filter();
}

/*
  start an autotune
 */
void AP_YawController::autotune_start(void)
{
    if (autotune == nullptr && rate_control_enabled()) {
        // the autotuner needs a time constant. Use an assumed tconst of 0.5
        gains.tau.set(0.5);
        gains.rmax_pos.set(90);

        autotune = new AP_AutoTune(gains, AP_AutoTune::AUTOTUNE_YAW, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed yaw allocation");
            }
            failed_autotune_alloc = true;
        }
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

/*
  restore autotune gains
 */
void AP_YawController::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}
