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

AP_FW_Controller::AP_FW_Controller(const AP_FixedWing &parms, const AC_PID::Defaults &defaults, AP_AutoTune::ATType _autotune_type)
    : aparm(parms),
      rate_pid(defaults),
      autotune_type(_autotune_type)
{
    rate_pid.set_slew_limit_scale(45);
}

/*
  AC_PID based rate controller
*/
float AP_FW_Controller::_get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode)
{
    const float dt = AP::scheduler().get_loop_period_s();

    const float eas2tas = AP::ahrs().get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    const float rate = get_measured_rate();
    const float old_I = rate_pid.get_i();

    const bool underspeed = is_underspeed(aspeed);
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

    if (autotune != nullptr && autotune->running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

/*
 Function returns an equivalent control surface deflection in centi-degrees in the range from -4500 to 4500
*/
float AP_FW_Controller::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false, get_airspeed(), false);
}

void AP_FW_Controller::reset_I()
{
    rate_pid.reset_I();
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

