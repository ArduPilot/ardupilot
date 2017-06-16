#include "AC_PosControl_Compound.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
/*
const AP_Param::GroupInfo AC_PosControl_Compound::var_info[] = {
    // 0 was used for HOVER

    // @Param: _
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ACC_XY_FILT", 1, AC_PosControl_Compound, throttle_p, throttle_p.kP()),

    AP_GROUPEND
};
*/
AC_PosControl_Compound::AC_PosControl_Compound(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             AC_P& p_pos_z, AC_P& p_vel_z, AC_PID& pid_accel_z,
                             AC_P& p_pos_xy, AC_PI_2D& pi_vel_xy, AC_P& throttle_p) :
    AC_PosControl(ahrs, inav, motors, attitude_control, p_pos_z, p_vel_z, pid_accel_z, p_pos_xy, pi_vel_xy),
    _ahrs(ahrs),
    _motors(motors),
    _thrust_out(-1.0f),
    _use_thruster(true),
    _throttle_p(throttle_p),
    _radio_forward_in(-1.0f)
{}

//To-Do: enable radio passthrough for stick-auto modes like stabilize, alt-hold, etc...

void AC_PosControl_Compound::set_radio_passthrough_forward_thruster(float forward_radio_passthrough)
{
    _radio_forward_in = constrain_float(forward_radio_passthrough,-1.0f, 1.0f);
    _motors.set_forward(_radio_forward_in);
}

void AC_PosControl_Compound::set_use_thruster(bool use_thruster)
{
    _use_thruster = use_thruster;

    if (!_use_thruster)
      _motors.set_forward(-1.0f);
}

void AC_PosControl_Compound::init_takeoff()
{
    AC_PosControl::init_takeoff();
    set_use_thruster(false);
}
// rate_to_accel_xy - horizontal desired rate to desired acceleration
///    converts desired velocities in lat/lon directions to accelerations in lat/lon frame
/*
void AC_PosControl_Compound::rate_to_accel_xy(float dt, float ekfNavVelGainScaler)
{
    Vector2f vel_xy_p, vel_xy_i;

    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_xy) {
        _vel_last.x = _vel_target.x;
        _vel_last.y = _vel_target.y;
        _flags.reset_rate_to_accel_xy = false;
    }

    // check if vehicle velocity is being overridden
    if (_flags.vehicle_horiz_vel_override) {
        _flags.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_horiz_vel.x = _inav.get_velocity().x;
        _vehicle_horiz_vel.y = _inav.get_velocity().y;
    }

    // feed forward desired acceleration calculation
    if (dt > 0.0f) {
    	if (!_flags.freeze_ff_xy) {
    		_accel_feedforward.x = (_vel_target.x - _vel_last.x)/dt;
    		_accel_feedforward.y = (_vel_target.y - _vel_last.y)/dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_xy = false;
    	}
    } else {
    	_accel_feedforward.x = 0.0f;
    	_accel_feedforward.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;
    _vel_last.y = _vel_target.y;

    // calculate velocity error
    _vel_error.x = _vel_target.x - _vehicle_horiz_vel.x;
    _vel_error.y = _vel_target.y - _vehicle_horiz_vel.y;

    // call pi controller
    _pi_vel_xy.set_input(_vel_error);

    // get p
    vel_xy_p = _pi_vel_xy.get_p();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        vel_xy_i = _pi_vel_xy.get_i();
    } else {
        vel_xy_i = _pi_vel_xy.get_i_shrink();
    }

    // combine feed forward accel with PID output from velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    _accel_target.x = _accel_feedforward.x + (vel_xy_p.x + vel_xy_i.x) * ekfNavVelGainScaler;
    _accel_target.y = _accel_feedforward.y + (vel_xy_p.y + vel_xy_i.y) * ekfNavVelGainScaler;
}
*/

// Just some addition of use Forward Thruster when certain amount of accel_forward is requested.
void AC_PosControl_Compound::accel_to_lean_angles(float dt, float ekfNavVelGainScaler, bool use_althold_lean_angle)
{
    float accel_total;                          // total acceleration in cm/s/s
    float accel_right, accel_forward;
    float lean_angle_max = _attitude_control.lean_angle_max();
    float accel_max = POSCONTROL_ACCEL_XY_MAX;

    // limit acceleration if necessary
    if (use_althold_lean_angle) {
        accel_max = MIN(accel_max, GRAVITY_MSS * 100.0f * tanf(ToRad(constrain_float(_attitude_control.get_althold_lean_angle_max(),1000,8000)/100.0f)));
    }

    // scale desired acceleration if it's beyond acceptable limit
    accel_total = norm(_accel_target.x, _accel_target.y);
    if (accel_total > accel_max && accel_total > 0.0f) {
        _accel_target.x = accel_max * _accel_target.x/accel_total;
        _accel_target.y = accel_max * _accel_target.y/accel_total;
        _limit.accel_xy = true;     // unused
    } else {
        // reset accel limit flag
        _limit.accel_xy = false;
    }

    // reset accel to current desired acceleration
    if (_flags.reset_accel_to_lean_xy) {
        _accel_target_jerk_limited.x = _accel_target.x;
        _accel_target_jerk_limited.y = _accel_target.y;
        _accel_target_filter.reset(Vector2f(_accel_target.x, _accel_target.y));
        _flags.reset_accel_to_lean_xy = false;
    }

    // apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
    float max_delta_accel = dt * _jerk_cmsss;

    Vector2f accel_in(_accel_target.x, _accel_target.y);
    Vector2f accel_change = accel_in-_accel_target_jerk_limited;
    float accel_change_length = accel_change.length();

    if(accel_change_length > max_delta_accel) {
        accel_change *= max_delta_accel/accel_change_length;
    }
    _accel_target_jerk_limited += accel_change;

    // lowpass filter on NE accel
    _accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f*ekfNavVelGainScaler));
    Vector2f accel_target_filtered = _accel_target_filter.apply(_accel_target_jerk_limited, dt);

    // rotate accelerations into body forward-right frame
    accel_forward = accel_target_filtered.x*_ahrs.cos_yaw() + accel_target_filtered.y*_ahrs.sin_yaw();
    accel_right = -accel_target_filtered.x*_ahrs.sin_yaw() + accel_target_filtered.y*_ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    // if much accel_forward is requested use forward thruster.
    _pitch_target = constrain_float(atanf(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);

    float cos_pitch_target = cosf(_pitch_target*M_PI/18000);
    _roll_target = constrain_float(atanf(accel_right*cos_pitch_target/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);

    //To-Do : Do not allow use thruster which requires level of precision like circle, land modes, ...
    // To-Do 2 : Possibly There would be optimal ways to mix pitch down and rear throttle controller.
    if (_use_thruster)
    {
      if (accel_forward >= 0.0f)
        {
            use_althold_lean_angle = false;
            _pitch_target = 0.0f; // do not allow pitch down to accelerate

            run_auxiliary_thruster_controller(accel_forward);
        }
      else
      {
            use_althold_lean_angle = true;
            run_auxiliary_thruster_controller(0.0f);
            _motors.set_forward(-1.0f);
      }
    }

}

// Forward thruster controller command from requested Position/Velocity/Acceleration command.
void AC_PosControl_Compound::run_auxiliary_thruster_controller(float accel_forward)
{
     if (accel_forward > 0.0f)
     {
      //Vector3f accel_NED = _ahrs.get_accel_ef_blended();
      //float accel_error = 0.01f * accel_forward - accel_NED.x; // accel error in m/s/s this would be noisy controller
      //float radio_feedforward = _radio_forward_in - 0.5; // decelerate if less than 50% forward radio channel / accelerate if more than 50% RC-in
      _thrust_out = 0.01f * _throttle_p.get_p(accel_forward); // simple P controller in m/s/s
      //To-do : Implement accel forward to throttle controller like fixed-wing
      _motors.set_forward(_thrust_out);
     }
     else
     {
       _motors.set_forward(-1.0f);
     }
}
