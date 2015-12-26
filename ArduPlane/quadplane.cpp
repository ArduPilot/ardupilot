// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPINFO(motors, "M_", 1, QuadPlane, AP_MotorsQuad),

    // @Param: RT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
    AP_SUBGROUPINFO(pid_rate_roll, "RT_RLL_", 2, QuadPlane, AC_PID),

    // @Param: RT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
    AP_SUBGROUPINFO(pid_rate_pitch, "RT_PIT_", 3, QuadPlane, AC_PID),

    // @Param: RT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.150 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.05
    // @Increment: 0.01
    // @User: Standard

    // @Param: RT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard
    AP_SUBGROUPINFO(pid_rate_yaw, "RT_YAW_", 4, QuadPlane, AC_PID),

    // P controllers
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_roll, "STB_R_", 5, QuadPlane, AC_P),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_pitch, "STB_P_", 6, QuadPlane, AC_P),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_yaw,   "STB_Y_", 7, QuadPlane, AC_P),

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    AP_SUBGROUPINFO(attitude_control,  "A_", 8, QuadPlane, AC_AttitudeControl),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: Centi-degrees
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 9, QuadPlane, aparm.angle_max, 4500),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: milli-seconds
    // @Range: 0 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 10, QuadPlane, transition_time_ms, 5000),

    // @Param: PZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(p_alt_hold, "PZ_", 11, QuadPlane, AC_P),

    // @Param: PXY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(p_pos_xy,   "PXY_", 12, QuadPlane, AC_P),

    // @Param: VXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced
    AP_SUBGROUPINFO(pi_vel_xy, "VXY_",  13, QuadPlane, AC_PI_2D),

    // @Param: VZ_P
    // @DisplayName: Velocity (vertical) P gain
    // @Description: Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(p_vel_z,   "VZ_", 14, QuadPlane, AC_P),

    // @Param: AZ_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: AZ_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: AZ_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Standard

    // @Param: AZ_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: AZ_FILT_HZ
    // @DisplayName: Throttle acceleration filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(pid_accel_z, "AZ_", 15, QuadPlane, AC_PID),

    // @Group: P_
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPINFO(pos_control, "P", 16, QuadPlane, AC_PosControl),

    // @Param: VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 17, QuadPlane, pilot_velocity_z_max, 250),
    
    // @Param: ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",  18, QuadPlane, pilot_accel_z,  250),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPINFO(wp_nav, "WP_",  19, QuadPlane, AC_WPNav),
    
    AP_GROUPEND
};

QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void QuadPlane::setup(void)
{
    motors.set_frame_orientation(AP_MOTORS_QUADPLANE);
    motors.set_throttle_range(0, 1000, 2000);
    motors.set_hover_throttle(500);
    motors.set_update_rate(490);
    motors.set_interlock(true);
    motors.set_loop_rate(plane.ins.get_sample_rate());
    attitude_control.set_dt(plane.ins.get_loop_delta_t());
    pid_rate_roll.set_dt(plane.ins.get_loop_delta_t());
    pid_rate_pitch.set_dt(plane.ins.get_loop_delta_t());
    pid_rate_yaw.set_dt(plane.ins.get_loop_delta_t());
    pid_accel_z.set_dt(plane.ins.get_loop_delta_t());
    pos_control.set_dt(plane.ins.get_loop_delta_t());
}

// init quadplane stabilize mode 
void QuadPlane::init_stabilize(void)
{
    throttle_wait = false;
}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                        plane.nav_pitch_cd,
                                                                        get_pilot_desired_yaw_rate_cds(),
                                                                        smoothing_gain);

    attitude_control.set_throttle_out(throttle_in, true, 0);
}

// quadplane stabilize mode
void QuadPlane::control_stabilize(void)
{
    int16_t pilot_throttle_scaled = plane.channel_throttle->control_in * 10;
    hold_stabilize(pilot_throttle_scaled);

}

// init quadplane hover mode 
void QuadPlane::init_hover(void)
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control.set_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate)
{
    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control.set_accel_z(pilot_accel_z);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                        plane.nav_pitch_cd,
                                                                        get_pilot_desired_yaw_rate_cds(),
                                                                        smoothing_gain);

    // call position controller
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, plane.G_Dt, false);
    pos_control.update_z_controller();

}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{
    if (throttle_wait) {
        attitude_control.set_throttle_out_unstabilized(0, true, 0);
        pos_control.relax_alt_hold_controllers(0);
    } else {
        hold_hover(get_pilot_desired_climb_rate_cms());
    }
}

void QuadPlane::init_loiter(void)
{
    // set target to current position
    wp_nav.init_loiter_target();

    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control.set_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        attitude_control.set_throttle_out_unstabilized(0, true, 0);
        pos_control.relax_alt_hold_controllers(0);
        wp_nav.init_loiter_target();
        return;
    }
    
    // initialize vertical speed and acceleration
    pos_control.set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control.set_accel_z(pilot_accel_z);

    // process pilot's roll and pitch input
    wp_nav.set_pilot_desired_acceleration(plane.channel_roll->control_in,
                                          plane.channel_pitch->control_in);

    // Update EKF speed limit - used to limit speed when we are using optical flow
    float ekfGndSpdLimit, ekfNavVelGainScaler;    
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    
    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(),
                                                                 wp_nav.get_pitch(),
                                                                 get_pilot_desired_yaw_rate_cds());

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav.get_roll();
    plane.nav_pitch_cd = wp_nav.get_pitch();

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate_ff(get_pilot_desired_climb_rate_cms(), plane.G_Dt, false);
    pos_control.update_z_controller();
}

/*
  get desired yaw rate in cd/s
 */
float QuadPlane::get_pilot_desired_yaw_rate_cds(void)
{
    const float yaw_rate_max_dps = 100;
    return plane.channel_rudder->norm_input() * 100 * yaw_rate_max_dps;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void)
{
    return pilot_velocity_z_max * (plane.channel_throttle->control_in - 50) / 50.0;
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    if (plane.channel_throttle->control_in >= 10 ||
        plane.is_flying()) {
        throttle_wait = false;
    } else {
        throttle_wait = true;        
    }
}
    
// set motor arming
void QuadPlane::set_armed(bool armed)
{
    motors.armed(armed);
    if (armed) {
        motors.enable();
    }
}


/*
  update for transition from quadplane to fixed wing mode
 */
void QuadPlane::update_transition(void)
{
    if (plane.control_mode == MANUAL) {
        // in manual mode quad motors are always off
        motors.output_min();
        transition_state = TRANSITION_DONE;
        return;
    }

    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Transition airspeed wait");
            transition_start_ms = millis();
        }

        float aspeed;
        if (ahrs.airspeed_estimate(&aspeed) && aspeed > plane.aparm.airspeed_min) {
            transition_start_ms = millis();
            transition_state = TRANSITION_TIMER;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Transition airspeed reached %.1f", aspeed);
        }
        hold_hover(0);
        break;
    }
        
    case TRANSITION_TIMER: {
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        if (millis() - transition_start_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Transition done");
        }
        float throttle_scaled = last_throttle * (transition_time_ms - (millis() - transition_start_ms)) / (float)transition_time_ms;
        if (throttle_scaled < 0) {
            throttle_scaled = 0;
        }
        hold_stabilize(throttle_scaled);
        motors.output();
        break;
    }

    case TRANSITION_DONE:
        motors.output_min();
        break;
    }
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (plane.control_mode != QSTABILIZE &&
        plane.control_mode != QHOVER &&
        plane.control_mode != QLOITER) {
        update_transition();
    } else {
        // run low level rate controllers
        attitude_control.rate_controller_run();

        // output to motors
        motors.output();
        transition_start_ms = 0;
        transition_state = TRANSITION_AIRSPEED_WAIT;
        last_throttle = motors.get_throttle();
    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait && plane.channel_throttle->control_in > 10) {
        throttle_wait = false;
    }
}
