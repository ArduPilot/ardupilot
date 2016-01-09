// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming quad motors on outputs 5 to 8
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPPTR(motors, "M_", 2, QuadPlane, AP_MotorsQuad),

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
    AP_SUBGROUPINFO(pid_rate_roll, "RT_RLL_", 3, QuadPlane, AC_PID),

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
    AP_SUBGROUPINFO(pid_rate_pitch, "RT_PIT_", 4, QuadPlane, AC_PID),

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
    AP_SUBGROUPINFO(pid_rate_yaw, "RT_YAW_", 5, QuadPlane, AC_PID),

    // P controllers
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_roll, "STB_R_", 6, QuadPlane, AC_P),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_pitch, "STB_P_", 7, QuadPlane, AC_P),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard
    AP_SUBGROUPINFO(p_stabilize_yaw,   "STB_Y_", 8, QuadPlane, AC_P),

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    AP_SUBGROUPPTR(attitude_control,  "A_", 9, QuadPlane, AC_AttitudeControl),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: Centi-degrees
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 4500),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: milli-seconds
    // @Range: 0 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // @Param: PZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(p_alt_hold, "PZ_", 12, QuadPlane, AC_P),

    // @Param: PXY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(p_pos_xy,   "PXY_", 13, QuadPlane, AC_P),

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
    AP_SUBGROUPINFO(pi_vel_xy, "VXY_",  14, QuadPlane, AC_PI_2D),

    // @Param: VZ_P
    // @DisplayName: Velocity (vertical) P gain
    // @Description: Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(p_vel_z,   "VZ_", 15, QuadPlane, AC_P),

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
    AP_SUBGROUPINFO(pid_accel_z, "AZ_", 16, QuadPlane, AC_PID),

    // @Group: P_
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 18, QuadPlane, pilot_velocity_z_max, 250),
    
    // @Param: ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",  19, QuadPlane, pilot_accel_z,  250),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: Hz
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN_PWM", 22, QuadPlane, thr_min_pwm, 1000),

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: Hz
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX_PWM", 23, QuadPlane, thr_max_pwm, 2000),

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. Zero means no assistance except during transition
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist_speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate in degrees/second
    // @Units: degrees/second
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_RATE_MAX", 25, QuadPlane, yaw_rate_max, 100),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LAND_SPEED", 26, QuadPlane, land_speed_cms, 50),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt, 6),

    // @Param: THR_MID
    // @DisplayName: Throttle Mid Position
    // @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
    // @User: Standard
    // @Range: 300 700
    // @Units: Percent*10
    // @Increment: 1
    AP_GROUPINFO("THR_MID", 28, QuadPlane, throttle_mid, 500),
    
    AP_GROUPEND
};

QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool QuadPlane::setup(void)
{
    uint16_t mask;
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }

    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Not enough memory for quadplane");
        goto failed;
    }
    
    // setup default motors for X frame
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor1, CH_5);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor2, CH_6);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor3, CH_7);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor4, CH_8);
    
    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    motors = new AP_MotorsQuad(plane.ins.get_sample_rate());
    if (!motors) {
        hal.console->printf("Unable to allocate motors\n");
        goto failed;
    }
    AP_Param::load_object_from_eeprom(motors, motors->var_info);
    attitude_control = new AC_AttitudeControl_Multi(ahrs, aparm, *motors,
                                                    p_stabilize_roll, p_stabilize_pitch, p_stabilize_yaw,
                                                    pid_rate_roll, pid_rate_pitch, pid_rate_yaw);
    if (!attitude_control) {
        hal.console->printf("Unable to allocate attitude_control\n");
        goto failed;
    }
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = new AC_PosControl(ahrs, inertial_nav, *motors, *attitude_control,
                                    p_alt_hold, p_vel_z, pid_accel_z,
                                    p_pos_xy, pi_vel_xy);
    if (!pos_control) {
        hal.console->printf("Unable to allocate pos_control\n");
        goto failed;
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = new AC_WPNav(inertial_nav, ahrs, *pos_control, *attitude_control);
    if (!pos_control) {
        hal.console->printf("Unable to allocate wp_nav\n");
        goto failed;
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    motors->set_frame_orientation(AP_MOTORS_X_FRAME);
    motors->Init();
    motors->set_throttle_range(0, thr_min_pwm, thr_max_pwm);
    motors->set_hover_throttle(throttle_mid);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    attitude_control->set_dt(plane.ins.get_loop_delta_t());
    pid_rate_roll.set_dt(plane.ins.get_loop_delta_t());
    pid_rate_pitch.set_dt(plane.ins.get_loop_delta_t());
    pid_rate_yaw.set_dt(plane.ins.get_loop_delta_t());
    pid_accel_z.set_dt(plane.ins.get_loop_delta_t());
    pos_control->set_dt(plane.ins.get_loop_delta_t());

    // setup the trim of any motors used by AP_Motors so px4io
    // failsafe will disable motors
    mask = motors->get_motor_mask();
    for (uint8_t i=0; i<16; i++) {
        if (mask & 1U<<i) {
            RC_Channel *ch = RC_Channel::rc_channel(i);
            if (ch != nullptr) {
                ch->radio_trim = thr_min_pwm;
            }
        }
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // redo failsafe mixing on px4
    plane.setup_failsafe_mixing();
#endif
    
    transition_state = TRANSITION_DONE;

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "QuadPlane initialised");
    initialised = true;
    return true;
    
failed:
    initialised = false;
    enable.set(0);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "QuadPlane setup failed");
    return false;
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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                         plane.nav_pitch_cd,
                                                                         get_pilot_desired_yaw_rate_cds(),
                                                                         smoothing_gain);

    if (throttle_in <= 0) {
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
    } else {
        attitude_control->set_throttle_out(throttle_in, true, 0);
    }
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
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    pos_control->set_alt_target(inertial_nav.get_altitude());
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate)
{
    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                         plane.nav_pitch_cd,
                                                                         get_pilot_desired_yaw_rate_cds(),
                                                                         smoothing_gain);

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, plane.G_Dt, false);
    pos_control->update_z_controller();

}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{
    if (throttle_wait) {
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
    } else {
        hold_hover(get_pilot_desired_climb_rate_cms());
    }
}

void QuadPlane::init_loiter(void)
{
    // set target to current position
    wp_nav->init_loiter_target();

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    pos_control->set_alt_target(inertial_nav.get_altitude());
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}


// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (motors->get_throttle() > 200 && !motors->limit.throttle_lower) {
        return true;
    }
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
    bool motor_at_lower_limit = motors->limit.throttle_lower && motors->is_throttle_mix_min();
    if (motors->get_throttle() < 10) {
        motor_at_lower_limit = true;
    }
    if (!motor_at_lower_limit) {
        motors_lower_limit_start_ms = 0;
    }
    if (motor_at_lower_limit && motors_lower_limit_start_ms == 0) {
        motors_lower_limit_start_ms = millis();
    }
    bool relax_loiter = motors_lower_limit_start_ms != 0 && (millis() - motors_lower_limit_start_ms) > 1000;
    return relax_loiter;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
        wp_nav->init_loiter_target();
        return;
    }


    if (should_relax()) {
        wp_nav->loiter_soften_for_landing();
    }

    if (millis() - last_loiter_ms > 500) {
        wp_nav->init_loiter_target();
    }
    last_loiter_ms = millis();
    
    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // process pilot's roll and pitch input
    wp_nav->set_pilot_desired_acceleration(plane.channel_roll->control_in,
                                           plane.channel_pitch->control_in);

    // Update EKF speed limit - used to limit speed when we are using optical flow
    float ekfGndSpdLimit, ekfNavVelGainScaler;    
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    
    // run loiter controller
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(),
                                                                  wp_nav->get_pitch(),
                                                                  get_pilot_desired_yaw_rate_cds());

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(get_pilot_desired_climb_rate_cms(), plane.G_Dt, false);
    pos_control->update_z_controller();
}

/*
  get desired yaw rate in cd/s
 */
float QuadPlane::get_pilot_desired_yaw_rate_cds(void)
{
    float yaw_cds = 0;
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_yaw_rate_cds();
    }
    if (plane.channel_throttle->control_in <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }

    // add in rudder input
    yaw_cds += plane.channel_rudder->norm_input() * 100 * yaw_rate_max;
    return yaw_cds;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void)
{
    if (plane.failsafe.ch3_failsafe || plane.failsafe.ch3_counter > 0) {
        // descend at 0.5m/s for now
        return -50;
    }
    uint16_t dead_zone = plane.channel_throttle->get_dead_zone();
    uint16_t trim = (plane.channel_throttle->radio_max + plane.channel_throttle->radio_min)/2;
    return pilot_velocity_z_max * plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) / 100.0f;
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
    if (!initialised) {
        return;
    }
    motors->armed(armed);
    if (armed) {
        motors->enable();
    }
}


/*
  estimate desired climb rate for assistance (in cm/s)
 */
float QuadPlane::assist_climb_rate_cms(void)
{
    if (plane.auto_throttle_mode) {
        // ask TECS for its desired climb rate
        return plane.TECS_controller.get_height_rate_demand()*100;
    }
    // otherwise estimate from pilot input
    float climb_rate = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
    climb_rate *= plane.channel_throttle->control_in;
    return climb_rate;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_yaw_rate_cds(void)
{
    float aspeed;
    if (!ahrs.airspeed_estimate(&aspeed) || aspeed < plane.aparm.airspeed_min) {
        aspeed = plane.aparm.airspeed_min;
    }
    if (aspeed < 1) {
        aspeed = 1;
    }
    float yaw_rate = degrees(GRAVITY_MSS * tanf(radians(plane.nav_roll_cd*0.01f))/aspeed) * 100;
    return yaw_rate;
}

/*
  update for transition from quadplane to fixed wing mode
 */
void QuadPlane::update_transition(void)
{
    if (plane.control_mode == MANUAL ||
        plane.control_mode == ACRO ||
        plane.control_mode == TRAINING) {
        // in manual modes quad motors are always off
        motors->output_min();
        transition_state = TRANSITION_DONE;
        return;
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);
    
    /*
      see if we should provide some assistance
     */
    if (have_airspeed && aspeed < assist_speed &&
        (plane.auto_throttle_mode ||
         plane.channel_throttle->control_in>0 ||
         plane.is_flying())) {
        // the quad should provide some assistance to the plane
        transition_state = TRANSITION_AIRSPEED_WAIT;
        transition_start_ms = millis();
        assisted_flight = true;
    } else {
        assisted_flight = false;
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = millis();
        }

        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !assisted_flight) {
            transition_start_ms = millis();
            transition_state = TRANSITION_TIMER;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", aspeed);
        } else if (plane.auto_throttle_mode) {
            // force pitch to zero while building up airspeed
            plane.nav_pitch_cd = 0;
        }
        assisted_flight = true;
        hold_hover(assist_climb_rate_cms());
        attitude_control->rate_controller_run();
        motors->output();
        last_throttle = motors->get_throttle();
        break;
    }
        
    case TRANSITION_TIMER: {
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        if (millis() - transition_start_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition done");
        }
        float throttle_scaled = last_throttle * (transition_time_ms - (millis() - transition_start_ms)) / (float)transition_time_ms;
        if (throttle_scaled < 0) {
            throttle_scaled = 0;
        }
        assisted_flight = true;
        hold_stabilize(throttle_scaled);
        attitude_control->rate_controller_run();
        motors->output();
        break;
    }

    case TRANSITION_DONE:
        motors->output_min();
        break;
    }
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }

    bool quad_mode = (plane.control_mode == QSTABILIZE ||
                      plane.control_mode == QHOVER ||
                      plane.control_mode == QLOITER ||
                      in_vtol_auto());
    
    if (!quad_mode) {
        update_transition();
    } else {
        assisted_flight = false;
        
        // run low level rate controllers
        attitude_control->rate_controller_run();

        // output to motors
        motors->output();
        transition_start_ms = 0;
        if (throttle_wait && !plane.is_flying()) {
            transition_state = TRANSITION_DONE;
        } else {
            transition_state = TRANSITION_AIRSPEED_WAIT;
        }
        last_throttle = motors->get_throttle();
    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.channel_throttle->control_in > 10 ||
         plane.failsafe.ch3_failsafe ||
         plane.failsafe.ch3_counter>0)) {
        throttle_wait = false;
    }
}

/*
  update control mode for quadplane modes
 */
void QuadPlane::control_run(void)
{
    if (!initialised) {
        return;
    }

    switch (plane.control_mode) {
    case QSTABILIZE:
        control_stabilize();
        break;
    case QHOVER:
        control_hover();
        break;
    case QLOITER:
        control_loiter();
        break;
    default:
        break;
    }
    // we also stabilize using fixed wing surfaces
    float speed_scaler = plane.get_speed_scaler();
    plane.stabilize_roll(speed_scaler);
    plane.stabilize_pitch(speed_scaler);
}

/*
  enter a quadplane mode
 */
bool QuadPlane::init_mode(void)
{
    if (!setup()) {
        return false;
    }
    if (!initialised) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "QuadPlane mode refused");        
        return false;
    }
    switch (plane.control_mode) {
    case QSTABILIZE:
        init_stabilize();
        break;
    case QHOVER:
        init_hover();
        break;
    case QLOITER:
        init_loiter();
        break;
    default:
        break;
    }
    return true;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(const mavlink_command_long_t &packet)
{
    if (!available()) {
        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "VTOL not available");
        return MAV_RESULT_FAILED;
    }
    if (plane.control_mode != AUTO) {
        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return MAV_RESULT_FAILED;
    }
    switch ((uint8_t)packet.param1) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        return MAV_RESULT_ACCEPTED;
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;
        return MAV_RESULT_ACCEPTED;
    }

    plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return MAV_RESULT_FAILED;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void)
{
    if (plane.control_mode != AUTO) {
        return false;
    }
    if (plane.auto_state.vtol_mode) {
        return true;
    }
    switch (plane.mission.get_current_nav_cmd().id) {
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    default:
        return false;
    }
}

/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(const Location &loc)
{
    if (!setup()) {
        return;
    }
    Location origin = inertial_nav.get_origin();
    Vector2f diff2d;
    Vector3f target;
    diff2d = location_diff(origin, loc);
    target.x = diff2d.x * 100;
    target.y = diff2d.y * 100;
    target.z = loc.alt - origin.alt;

    if (!locations_are_same(loc, last_auto_target) ||
        loc.alt != last_auto_target.alt ||
        millis() - last_loiter_ms > 500) {
        wp_nav->set_wp_destination(target);
        last_auto_target = loc;
    }
    last_loiter_ms = millis();
    
    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_TAKEOFF ||
        (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_LAND &&
         land_state >= QLAND_FINAL)) {
        /*
          we need to use the loiter controller for final descent as
          the wpnav controller takes over the descent rate control
         */
        float ekfGndSpdLimit, ekfNavVelGainScaler;    
        ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    
        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                             plane.nav_pitch_cd,
                                                                             0,
                                                                             smoothing_gain);
    } else {
        float aspeed;
        int pitch_limit_cd = plane.aparm.pitch_limit_max_cd;
        if (assist_speed > 0 && ahrs.airspeed_estimate(&aspeed) && aspeed < assist_speed) {
            if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_LAND &&
                land_state == QLAND_POSITION) {
                // when starting the reposition limit the pitch for less dramatic slow down
                const float threshold = 0.5f * assist_speed;
                if (aspeed > threshold && plane.auto_state.wp_distance > 10 &&
                    !location_passed_point(plane.current_loc, plane.prev_WP_loc, plane.next_WP_loc)) {
                    float p = constrain_float((aspeed - threshold)/threshold, 0, 1);
                    pitch_limit_cd = p*plane.aparm.pitch_limit_max_cd + 500*(1-p);
                    plane.nav_pitch_cd = MIN(plane.nav_pitch_cd, pitch_limit_cd);
                }
            } else if (aspeed < assist_speed) {
                // while transitioning limit pitch to let forward motor gain speed
                pitch_limit_cd = 500;
            }
        }
        
        // run wpnav controller
        wp_nav->update_wpnav();

        if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_LAND && land_state >= QLAND_DESCEND) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_smooth(plane.nav_roll_cd,
                                                                                 plane.nav_pitch_cd,
                                                                                 0,
                                                                                 smoothing_gain);
        } else {
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(),
                                                               MIN(wp_nav->get_pitch(), pitch_limit_cd),
                                                               wp_nav->get_yaw(),
                                                               true);
        }
    }


    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    switch (plane.mission.get_current_nav_cmd().id) {
    case MAV_CMD_NAV_VTOL_LAND:
        if (land_state < QLAND_FINAL) {
            pos_control->set_alt_target_with_slew(wp_nav->get_loiter_target().z, plane.ins.get_loop_delta_t());
        } else {
            pos_control->set_alt_target_from_climb_rate(-land_speed_cms, plane.G_Dt, true);            
        }
        break;
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        pos_control->set_alt_target_from_climb_rate(100, plane.G_Dt, true);            
        break;
    default:
        pos_control->set_alt_target_from_climb_rate_ff(assist_climb_rate_cms(), plane.G_Dt, false);
        break;
    }
    
    pos_control->update_z_controller();
}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }
    plane.set_next_WP(cmd.content.location);
    plane.next_WP_loc.alt = plane.current_loc.alt + cmd.content.location.alt;
    throttle_wait = false;

    // set target to current position
    wp_nav->init_loiter_target();

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    pos_control->set_alt_target(inertial_nav.get_altitude());
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    return true;
}


/*
  start a VTOL landing
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }
    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.alt = plane.current_loc.alt;
    land_state = QLAND_POSITION;
    throttle_wait = false;    
    motors_lower_limit_start_ms = 0;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    return true;
}

/*
  check if a VTOL takeoff has completed
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }
    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition_state = TRANSITION_AIRSPEED_WAIT;
    return true;
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }
    if (land_state == QLAND_POSITION &&
        plane.auto_state.wp_distance < 2) {
        land_state = QLAND_DESCEND;
        plane.gcs_send_text(MAV_SEVERITY_INFO,"Land descend started");
        plane.set_next_WP(cmd.content.location);        
    }

    if (should_relax()) {
        wp_nav->loiter_soften_for_landing();
    }

    // at land_final_alt begin final landing
    if (land_state == QLAND_DESCEND &&
        plane.current_loc.alt < plane.next_WP_loc.alt+land_final_alt*100) {
        land_state = QLAND_FINAL;
        pos_control->set_alt_target(inertial_nav.get_altitude());
        plane.gcs_send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    if (land_state == QLAND_FINAL &&
        (motors_lower_limit_start_ms != 0 &&
         millis() - motors_lower_limit_start_ms > 5000)) {
        plane.disarm_motors();
        land_state = QLAND_COMPLETE;
        plane.gcs_send_text(MAV_SEVERITY_INFO,"Land complete");
    }
    return false;
}
