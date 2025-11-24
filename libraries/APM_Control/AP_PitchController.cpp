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

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {

    // @Param: 2SRV_TCONST
    // @DisplayName: Pitch Time Constant
    // @Description: Time constant in seconds from demanded to achieved pitch angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
    // @Range: 0.4 1.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2SRV_TCONST",      0, AP_PitchController, gains.tau,       0.5f),

    // index 1 to 3 reserved for old PID values

    // @Param: 2SRV_RMAX_UP
    // @DisplayName: Pitch up max rate
    // @Description: This sets the maximum nose up pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX_UP",     4, AP_PitchController, gains.rmax_pos,   0.0f),

    // @Param: 2SRV_RMAX_DN
    // @DisplayName: Pitch down max rate
    // @Description: This sets the maximum nose down pitch rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables the limit.
    // @Range: 0 100
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2SRV_RMAX_DN",     5, AP_PitchController, gains.rmax_neg,   0.0f),

    // @Param: 2SRV_RLL
    // @DisplayName: Roll compensation
    // @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.
    // @Range: 0.7 1.5
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("2SRV_RLL",      6, AP_PitchController, _roll_ff,        1.0f),

    // index 7, 8 reserved for old IMAX, FF

    // @Param: _RATE_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain. Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _RATE_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: _RATE_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _RATE_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _RATE_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID),

    AP_GROUPEND
};

AP_PitchController::AP_PitchController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.04,
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
    AP_AutoTune::ATType::AUTOTUNE_PITCH)
{
    AP_Param::setup_object_defaults(this, var_info);
}

float AP_PitchController::get_measured_rate() const
{
    return AP::ahrs().get_gyro().y;
}

float AP_PitchController::get_airspeed() const
{
    float aspeed;
    if (!AP::ahrs().airspeed_EAS(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    return aspeed;
}

bool AP_PitchController::is_underspeed(const float aspeed) const
{
    return aspeed <= 0.5*float(aparm.airspeed_min);
}

/*
  get the rate offset in degrees/second needed for pitch in body frame
  to maintain height in a coordinated turn.

  Also returns the inverted flag and the estimated airspeed in m/s for
  use by the rest of the pitch controller
 */
float AP_PitchController::_get_coordination_rate_offset(const float &aspeed, bool &inverted) const
{
    float rate_offset;
    float bank_angle = AP::ahrs().get_roll_rad();

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < radians(90))	{
        bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
    } else {
        inverted = true;
        if (bank_angle > 0.0f) {
            bank_angle = constrain_float(bank_angle,radians(100),radians(180));
        } else {
            bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
        }
    }
    const AP_AHRS &_ahrs = AP::ahrs();
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.get_pitch_rad())*fabsf(degrees((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1))) * tanf(bank_angle) * sinf(bank_angle))) * _roll_ff;
    }
    if (inverted) {
        rate_offset = -rate_offset;
    }
    return rate_offset;
}

// Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
// A positive demand is up
// Inputs are:
// 1) demanded pitch angle in centi-degrees
// 2) control gain scaler = scaling_speed / aspeed
// 3) boolean which is true when stabilise mode is active
// 4) minimum FBW airspeed (metres/sec)
// 5) maximum FBW airspeed (metres/sec)
//
float AP_PitchController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode)
{
    // Calculate offset to pitch rate demand required to maintain pitch angle whilst banking
    // Calculate ideal turn rate from bank angle and airspeed assuming a level coordinated turn
    // Pitch rate offset is the component of turn rate about the pitch axis
    float rate_offset;
    bool inverted;

    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }

    const float aspeed = get_airspeed();

    rate_offset = _get_coordination_rate_offset(aspeed, inverted);

    // Calculate the desired pitch rate (deg/sec) from the angle error
    angle_err_deg = angle_err * 0.01;
    float desired_rate = angle_err_deg / gains.tau;

    // limit the maximum pitch rate demand. Don't apply when inverted
    // as the rates will be tuned when upright, and it is common that
    // much higher rates are needed inverted
    if (!inverted) {
        desired_rate += rate_offset;
        if (gains.rmax_neg && desired_rate < -gains.rmax_neg) {
            desired_rate = -gains.rmax_neg;
        } else if (gains.rmax_pos && desired_rate > gains.rmax_pos) {
            desired_rate = gains.rmax_pos;
        }
    } else {
        // Make sure not to invert the turn coordination offset
        desired_rate = -desired_rate + rate_offset;
    }

    /*
      when we are past the users defined roll limit for the aircraft
      our priority should be to bring the aircraft back within the
      roll limit. Using elevator for pitch control at large roll
      angles is ineffective, and can be counter productive as it
      induces earth-frame yaw which can reduce the ability to roll. We
      linearly reduce pitch demanded rate when beyond the configured
      roll limit, reducing to zero at 90 degrees
    */
    const AP_AHRS &_ahrs = AP::ahrs();
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = MIN(aparm.roll_limit*100 + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        desired_rate *= (1 - roll_prop);
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator, aspeed, ground_mode);
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_PitchController::convert_pid()
{
    AP_Float &ff = rate_pid.ff();
    if (ff.configured()) {
        return;
    }

    float old_ff=0, old_p=1.0, old_i=0.3, old_d=0.08;
    int16_t old_imax = 3000;
    bool have_old = AP_Param::get_param_by_index(this, 1, AP_PARAM_FLOAT, &old_p);
    have_old |= AP_Param::get_param_by_index(this, 3, AP_PARAM_FLOAT, &old_i);
    have_old |= AP_Param::get_param_by_index(this, 2, AP_PARAM_FLOAT, &old_d);
    have_old |= AP_Param::get_param_by_index(this, 8, AP_PARAM_FLOAT, &old_ff);
    have_old |= AP_Param::get_param_by_index(this, 7, AP_PARAM_FLOAT, &old_imax);
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
