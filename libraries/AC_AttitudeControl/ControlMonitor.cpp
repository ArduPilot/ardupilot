/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

/*
  code to monitor and report on the rate controllers, allowing for
  notification of controller oscillation
 */


/*
  update a RMS estimate of controller state
 */
void AC_AttitudeControl::control_monitor_filter_pid(const DataFlash_Class::PID_Info &pid_info, float &rms)
{
    float value = sq(pid_info.P + pid_info.D + pid_info.FF);
    const float filter_constant = 0.99f;
    // we don't do the sqrt() here as it is quite expensive. That is
    // done when reporting a result
    rms = filter_constant * rms + (1.0f - filter_constant) * value;
}

/*
  update state in _control_monitor
 */
void AC_AttitudeControl::control_monitor_update(void)
{
    control_monitor_filter_pid(get_rate_roll_pid().get_pid_info(),  _control_monitor.rms_roll);
    control_monitor_filter_pid(get_rate_pitch_pid().get_pid_info(), _control_monitor.rms_pitch);
    control_monitor_filter_pid(get_rate_yaw_pid().get_pid_info(),   _control_monitor.rms_yaw);
}

/*
  log a CRTL message
 */
void AC_AttitudeControl::control_monitor_log(void)
{
    DataFlash_Class::instance()->Log_Write("CTRL", "TimeUS,RMSRoll,RMSPitch,RMSYaw", "Qfff",
                                           AP_HAL::micros64(),
                                           (double)sqrtf(_control_monitor.rms_roll),
                                           (double)sqrtf(_control_monitor.rms_pitch),
                                           (double)sqrtf(_control_monitor.rms_yaw));
}

/*
  return current controller RMS filter value for roll
 */
float AC_AttitudeControl::control_monitor_rms_output_roll(void) const
{
    return sqrtf(_control_monitor.rms_roll);
}

/*
  return current controller RMS filter value for pitch
 */
float AC_AttitudeControl::control_monitor_rms_output_pitch(void) const
{
    return sqrtf(_control_monitor.rms_pitch);
}

/*
  return current controller RMS filter value for yaw
 */
float AC_AttitudeControl::control_monitor_rms_output_yaw(void) const
{
    return sqrtf(_control_monitor.rms_yaw);
}
