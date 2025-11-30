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
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>

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
    // @Description: This sets the maximum roll rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables this limit.
    // @Range: 0 180
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX",   4, AP_RollController, gains.rmax_pos,       0),

    // index 5, 6 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain. Corrects in proportion to the difference between the desired roll rate vs actual roll rate
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
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
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

    // @Param: _RATE_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: _RATE_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _RATE_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _RATE_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),

    AP_GROUPEND
};

// constructor
AP_RollController::AP_RollController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.08,
        .i         = 0.15,
        .d         = 0.0,
        .ff        = 0.345,
        .imax      = 0.666,
        .filt_T_hz = 3.0,
        .filt_E_hz = 0.0,
        .filt_D_hz = 12.0,
        .srmax     = 150.0,
        .srtau     = 1.0
    },
    AP_AutoTune::ATType::AUTOTUNE_ROLL)
{
    AP_Param::setup_object_defaults(this, var_info);
}

float AP_RollController::get_measured_rate() const
{
    return AP::ahrs().get_gyro().x;
}

float AP_RollController::get_airspeed() const
{
    float aspeed;
    if (!AP::ahrs().airspeed_EAS(aspeed)) {
        // If no airspeed available use 0
        aspeed = 0.0;
    }
    return aspeed;
}

bool AP_RollController::is_underspeed(const float aspeed) const
{
    return aspeed <= float(aparm.airspeed_min);
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
float AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode)
{
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }

    // Calculate the desired roll rate (deg/sec) from the angle error
    angle_err_deg = angle_err * 0.01;
    float desired_rate = angle_err_deg/ gains.tau;

    /*
      prevent indecision in the roll controller when target roll is
      close to 180 degrees from the current roll
     */
    const float indecision_threshold_deg = 160;
    const float last_desired_rate = _pid_info.target;
    const float abs_angle_err_deg = fabsf(angle_err_deg);
    if (abs_angle_err_deg > indecision_threshold_deg &&
        angle_err_deg <= 180) {
        if (desired_rate * last_desired_rate < 0) {
            desired_rate = -desired_rate;
            // increase the desired rate in proportion to the extra
            // angle we are requesting
            const float new_angle_err_deg = abs_angle_err_deg + (180 - abs_angle_err_deg)*2;
            desired_rate *= new_angle_err_deg / abs_angle_err_deg;
        }
    }

    if (!in_recovery) {
        // Limit the demanded roll rate. When we are in a VTOL
        // recovery we don't apply the limit
        if (gains.rmax_pos && desired_rate < -gains.rmax_pos) {
            desired_rate = - gains.rmax_pos;
        } else if (gains.rmax_pos && desired_rate > gains.rmax_pos) {
            desired_rate = gains.rmax_pos;
        }
    }

    // the in_recovery flag is single loop only
    in_recovery = false;

    return _get_rate_out(desired_rate, scaler, disable_integrator, get_airspeed(), ground_mode);
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_RollController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
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
