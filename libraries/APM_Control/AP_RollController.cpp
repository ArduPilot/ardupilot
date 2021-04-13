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

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
    // @Param: 2SRV_TCONST
	// @DisplayName: Roll Time Constant
	// @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("2SRV_TCONST",      0, AP_RollController, gains.tau,       0.5f),

    // index 1 to 3 reserved for old PID values

    // @Param: 2SRV_RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: Maximum roll rate that the roll controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 180
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("2SRV_RMAX",   4, AP_RollController, gains.rmax_pos,       0),

    // index 5, 6 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),
    
    AP_GROUPEND
};

// constructor
AP_RollController::AP_RollController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms)
        : aparm(parms)
        , _ahrs(ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    rate_pid.set_slew_limit_scale(45);
}


/*
  AC_PID based rate controller
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
    const float dt = AP::scheduler().get_loop_period_s();
    const float eas2tas = _ahrs.get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    float rate_x = _ahrs.get_gyro().x;
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
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate_x * scaler * scaler, limit_I);

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

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate_x);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        autotune->update(pinfo, scaler, angle_err_deg);
    }
    
    // output is scaled to notional centidegrees of deflection
    return constrain_int32(out * 100, -4500, 4500);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) desired roll rate in degrees/sec
 2) control gain scaler = scaling_speed / aspeed
*/
int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}

/*
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }
	
    // Calculate the desired roll rate (deg/sec) from the angle error
    angle_err_deg = angle_err * 0.01;
    float desired_rate = angle_err_deg/ gains.tau;

    // Limit the demanded roll rate
    if (gains.rmax_pos && desired_rate < -gains.rmax_pos) {
        desired_rate = - gains.rmax_pos;
    } else if (gains.rmax_pos && desired_rate > gains.rmax_pos) {
        desired_rate = gains.rmax_pos;
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
    rate_pid.reset_I();
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_RollController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured_in_storage()) {
        return;
    }
    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax=3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 6, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 5, AP_PARAM_INT16, &old_imax);
    if (!have_old) {
        // none of the old gains were set
        return;
    }

    const float kp_ff = MAX((old_p - old_i * gains.tau) * gains.tau  - old_d, 0);
    rate_pid.ff().set_and_save(old_ff + kp_ff);
    rate_pid.kI().set_and_save_ifchanged(old_i * gains.tau);
    rate_pid.kP().set_and_save_ifchanged(old_d);
    rate_pid.kD().set_and_save_ifchanged(0);
    rate_pid.kIMAX().set_and_save_ifchanged(old_imax/4500.0);
}

/*
  start an autotune
 */
void AP_RollController::autotune_start(void)
{
    if (autotune == nullptr) {
        autotune = new AP_AutoTune(gains, AP_AutoTune::AUTOTUNE_ROLL, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed roll allocation");
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
void AP_RollController::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}
