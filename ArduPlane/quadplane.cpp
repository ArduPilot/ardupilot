// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPPTR(motors, "M_", 2, QuadPlane, AP_MOTORS_CLASS),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: Centi-degrees
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

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

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: Degrees
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

#if FRAME_CONFIG == MULTICOPTER_FRAME
    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Quad, 1:Hexa, 2:Octa, 3:OctaQuad, 4:Y6
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 30, QuadPlane, frame_class, 0),
#endif

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: Controls use of forward motor in vtol modes. If this is zero then the forward motor will not be used for position control in VTOL modes. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use QLOITER for position hold with the forward motor.
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // @Param: WVANE_GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This controls the tendency to yaw to face into the wind. A value of 0.1 is to start with and will give a slow turn into the wind. Use a value of 0.4 for more rapid response. The weathervaning works by turning into the direction of roll.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WVANE_GAIN", 33, QuadPlane, weathervane.gain, 0),

    // @Param: WVANE_MINROLL
    // @DisplayName: Weathervaning min roll
    // @Description: This set the minimum roll in degrees before active weathervaning will start. This may need to be larger if your aircraft has bad roll trim.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WVANE_MINROLL", 34, QuadPlane, weathervane.min_roll, 1),

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when the loiter target is reached
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // @Param: TILT_MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    AP_GROUPINFO("TILT_MASK", 37, QuadPlane, tilt.tilt_mask, 0),

    // @Param: TILT_RATE
    // @DisplayName: Tiltrotor tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor
    // @Units: degrees/second
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE", 38, QuadPlane, tilt.max_rate_dps, 40),

    // @Param: TILT_MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: degrees
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("TILT_MAX", 39, QuadPlane, tilt.max_angle_deg, 45),

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read http://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),
    
    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least 1 second. Set to zero to disable angle assistance.
    // @Units: degrees
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist_angle, 30),

    AP_GROUPEND
};

static const struct {
    const char *name;
    float value;
} defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FILT", 10.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FILT", 10.0 },
};

QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
}


// setup default motors for the frame class
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        RC_Channel_aux::set_aux_channel_default((RC_Channel_aux::Aux_servo_function_t)(RC_Channel_aux::k_motor1+i), CH_5+i);
    }
}
    

bool QuadPlane::setup(void)
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }
    float loop_delta_t = 1.0 / plane.scheduler.get_loop_rate_hz();
    
    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav)) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Not enough memory for quadplane");
        goto failed;
    }

#if FRAME_CONFIG == TRI_FRAME
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor1, CH_5);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor2, CH_6);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor4, CH_8);
    RC_Channel_aux::set_aux_channel_default(RC_Channel_aux::k_motor7, CH_11);
    motors = new AP_MOTORS_CLASS(plane.scheduler.get_loop_rate_hz());
#else
    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    switch ((enum frame_class)frame_class.get()) {
    case FRAME_CLASS_QUAD:
        setup_default_channels(4);
        motors = new AP_MotorsQuad(plane.scheduler.get_loop_rate_hz());
        break;
    case FRAME_CLASS_HEXA:
        setup_default_channels(6);
        motors = new AP_MotorsHexa(plane.scheduler.get_loop_rate_hz());
        break;
    case FRAME_CLASS_OCTA:
        setup_default_channels(8);
        motors = new AP_MotorsOcta(plane.scheduler.get_loop_rate_hz());
        break;
    case FRAME_CLASS_OCTAQUAD:
        setup_default_channels(8);
        motors = new AP_MotorsOctaQuad(plane.scheduler.get_loop_rate_hz());
        break;
    case FRAME_CLASS_Y6:
        setup_default_channels(7);
        motors = new AP_MotorsY6(plane.scheduler.get_loop_rate_hz());
        break;
    default:
        hal.console->printf("Unknown frame class %u\n", (unsigned)frame_class.get());
        goto failed;
    }
#endif // AP_MOTORS_CLASS
    if (!motors) {
        hal.console->printf("Unable to allocate motors\n");
        goto failed;
    }
    
    AP_Param::load_object_from_eeprom(motors, motors->var_info);
    attitude_control = new AC_AttitudeControl_Multi(ahrs, aparm, *motors, loop_delta_t);
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

    motors->set_frame_orientation(frame_type);
    motors->Init();
    motors->set_throttle_range(thr_min_pwm, thr_max_pwm);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    pid_accel_z.set_dt(loop_delta_t);
    pos_control->set_dt(loop_delta_t);
    attitude_control->parameter_sanity_check();

    // setup the trim of any motors used by AP_Motors so px4io
    // failsafe will disable motors
    for (uint8_t i=0; i<8; i++) {
        RC_Channel_aux::Aux_servo_function_t func = (RC_Channel_aux::Aux_servo_function_t)(RC_Channel_aux::k_motor1+i);
        RC_Channel_aux::set_servo_failsafe_pwm(func, thr_min_pwm);
        uint8_t chan;
        if (RC_Channel_aux::find_channel(func, chan)) {
            RC_Channel::rc_channel(chan)->set_radio_trim(thr_min_pwm);
        }
    }

#if HAVE_PX4_MIXER
    // redo failsafe mixing on px4
    plane.setup_failsafe_mixing();
#endif
    
    transition_state = TRANSITION_DONE;

    setup_defaults();
    
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "QuadPlane initialised");
    initialised = true;
    return true;
    
failed:
    initialised = false;
    enable.set(0);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "QuadPlane setup failed");
    return false;
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(defaults_table); i++) {
        if (!AP_Param::set_default_by_name(defaults_table[i].name, defaults_table[i].value)) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "QuadPlane setup failure for %s",
                                             defaults_table[i].name);
            AP_HAL::panic("quadplane bad default %s", defaults_table[i].name);
        }
    }

    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
}

// run ESC calibration
void QuadPlane::run_esc_calibration(void)
{
    if (!motors->armed()) {
        motors->set_throttle_passthrough_for_esc_calibration(0);
        AP_Notify::flags.esc_calibration = false;
        return;
    }
    if (!AP_Notify::flags.esc_calibration) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Starting ESC calibration");        
    }
    AP_Notify::flags.esc_calibration = true;
    switch (esc_calibration) {
    case 1:
        // throttle based calibration
        motors->set_throttle_passthrough_for_esc_calibration(plane.channel_throttle->get_control_in() * 0.01f);
        break;
    case 2:
        // full range calibration
        motors->set_throttle_passthrough_for_esc_calibration(1);
        break;
    }
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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                         plane.nav_pitch_cd,
                                                                         get_desired_yaw_rate_cds(),
                                                                         smoothing_gain);

    if (throttle_in <= 0) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        attitude_control->set_throttle_out(throttle_in, true, 0);
    }
}

// quadplane stabilize mode
void QuadPlane::control_stabilize(void)
{
    // special check for ESC calibration in QSTABILIZE
    if (esc_calibration != 0) {
        run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = plane.channel_throttle->get_control_in() / 100.0f;
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
    // motors use full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                         plane.nav_pitch_cd,
                                                                         get_desired_yaw_rate_cds(),
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
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
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

void QuadPlane::init_land(void)
{
    init_loiter();
    throttle_wait = false;
    poscontrol.state = QPOS_LAND_DESCEND;
    landing_detect.lower_limit_start_ms = 0;
}


// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (motors->get_throttle() > 0.01f && !motors->limit.throttle_lower) {
        return true;
    }
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
    bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }
    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
    }
    if (motor_at_lower_limit && landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = millis();
    }
    bool relax_loiter = landing_detect.lower_limit_start_ms != 0 &&
        (millis() - landing_detect.lower_limit_start_ms) > 1000;
    return relax_loiter;
}

// see if we are flying in vtol
bool QuadPlane::is_flying_vtol(void)
{
    if (!available()) {
        return false;
    }
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == QSTABILIZE || plane.control_mode == QHOVER || plane.control_mode == QLOITER) {
        // in manual flight modes only consider aircraft landed when pilot demanded throttle is zero
        return plane.channel_throttle->get_control_in() > 0;
    }
    if (in_vtol_mode() && millis() - landing_detect.lower_limit_start_ms > 5000) {
        // use landing detector
        return true;
    }
    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt. 
 */
float QuadPlane::landing_descent_rate_cms(float height_above_ground)
{
    float ret = linear_interpolate(land_speed_cms, wp_nav->get_speed_down(),
                                   height_above_ground,
                                   land_final_alt, land_final_alt+3);
    return ret;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
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

    // motors use full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);

    // process pilot's roll and pitch input
    wp_nav->set_pilot_desired_acceleration(plane.channel_roll->get_control_in(),
                                           plane.channel_pitch->get_control_in());

    // Update EKF speed limit - used to limit speed when we are using optical flow
    float ekfGndSpdLimit, ekfNavVelGainScaler;    
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    
    // run loiter controller
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(),
                                                                  wp_nav->get_pitch(),
                                                                  get_desired_yaw_rate_cds(),
                                                                  4.0f);

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();

    if (plane.control_mode == QLAND) {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        if (height_above_ground < land_final_alt && poscontrol.state < QPOS_LAND_FINAL) {
            poscontrol.state = QPOS_LAND_FINAL;
            // cut IC engine if enabled
            if (land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float descent_rate = (poscontrol.state == QPOS_LAND_FINAL)? land_speed_cms:landing_descent_rate_cms(height_above_ground);
        pos_control->set_alt_target_from_climb_rate(-descent_rate, plane.G_Dt, true);
        check_land_complete();
    } else {
        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(get_pilot_desired_climb_rate_cms(), plane.G_Dt, false);
    }
    pos_control->update_z_controller();
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void)
{
    if (plane.channel_throttle->get_control_in() <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }

    // add in rudder input
    return plane.channel_rudder->norm_input() * 100 * yaw_rate_max;
}

/*
  get overall desired yaw rate in cd/s
 */
float QuadPlane::get_desired_yaw_rate_cds(void)
{
    float yaw_cds = 0;
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_auto_yaw_rate_cds();
    }
    if (plane.channel_throttle->get_control_in() <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }
    // add in pilot input
    yaw_cds += get_pilot_input_yaw_rate_cds();

    // add in weathervaning
    yaw_cds += get_weathervane_yaw_rate_cds();
    
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
    uint16_t trim = (plane.channel_throttle->get_radio_max() + plane.channel_throttle->get_radio_min())/2;
    return pilot_velocity_z_max * plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) / 100.0f;
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    if (plane.channel_throttle->get_control_in() >= 10 ||
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
    float climb_rate;
    if (plane.auto_throttle_mode) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate = plane.altitude_error_cm / 10;
    } else {
        // otherwise estimate from pilot input
        climb_rate = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
        climb_rate *= plane.channel_throttle->get_control_in();
    }
    climb_rate = constrain_float(climb_rate, -wp_nav->get_speed_down(), wp_nav->get_speed_up());
    return climb_rate;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_auto_yaw_rate_cds(void)
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
  return true if the quadplane should provide stability assistance
 */
bool QuadPlane::assistance_needed(float aspeed)
{
    if (assist_speed <= 0) {
        // assistance disabled
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    if (aspeed < assist_speed) {
        // assistance due to Q_ASSIST_SPEED
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return true;
    }

    if (assist_angle <= 0) {
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }

    /*
      now check if we should provide assistance due to attitude error
     */

    const uint16_t allowed_envelope_error_cd = 500U;
    if (labs(ahrs.roll_sensor) <= plane.aparm.roll_limit_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor < plane.aparm.pitch_limit_max_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor > -(plane.aparm.pitch_limit_min_cd+allowed_envelope_error_cd)) {
        // we are inside allowed attitude envelope
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    
    uint32_t max_angle_cd = 100U*assist_angle;
    if ((labs(ahrs.roll_sensor - plane.nav_roll_cd) < max_angle_cd &&
         labs(ahrs.pitch_sensor - plane.nav_pitch_cd) < max_angle_cd)) {
        // not beyond angle error
        angle_error_start_ms = 0;
        in_angle_assist = false;
        return false;
    }
    if (angle_error_start_ms == 0) {
        angle_error_start_ms = AP_HAL::millis();
    }
    bool ret = (AP_HAL::millis() - angle_error_start_ms) >= 1000U;
    if (ret && !in_angle_assist) {
        in_angle_assist = true;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Angle assist r=%d p=%d",
                                         (int)(ahrs.roll_sensor/100),
                                         (int)(ahrs.pitch_sensor/100));
    }
    return ret;
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
        if (!tilt.motors_active) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            motors->output();
        }
        transition_state = TRANSITION_DONE;
        return;
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);
    
    /*
      see if we should provide some assistance
     */
    if (have_airspeed &&
        assistance_needed(aspeed) &&
        (plane.auto_throttle_mode ||
         plane.channel_throttle->get_control_in()>0 ||
         plane.is_flying())) {
        // the quad should provide some assistance to the plane
        if (transition_state != TRANSITION_AIRSPEED_WAIT) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
        }
        transition_state = TRANSITION_AIRSPEED_WAIT;
        transition_start_ms = millis();
        assisted_flight = true;
    } else {
        assisted_flight = false;
    }

    if (transition_state < TRANSITION_TIMER) {
        // set a single loop pitch limit in TECS
        if (plane.ahrs.groundspeed() < 3) {
            // until we have some ground speed limit to zero pitch
            plane.TECS_controller.set_pitch_max_limit(0);
        } else {
            plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
        }
    } else if (transition_state < TRANSITION_DONE) {
        plane.TECS_controller.set_pitch_max_limit((transition_pitch_max+1)*2);
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = millis();
        }

        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !assisted_flight) {
            transition_start_ms = millis();
            transition_state = TRANSITION_TIMER;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        assisted_flight = true;
        hold_hover(assist_climb_rate_cms());
        run_rate_controller();
        motors_output();
        last_throttle = motors->get_throttle();
        break;
    }
        
    case TRANSITION_TIMER: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        if (millis() - transition_start_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Transition done");
        }
        float trans_time_ms = (float)transition_time_ms.get();
        float throttle_scaled = last_throttle * (trans_time_ms - (millis() - transition_start_ms)) / trans_time_ms;
        if (throttle_scaled < 0) {
            throttle_scaled = 0;
        }
        assisted_flight = true;
        hold_stabilize(throttle_scaled);
        run_rate_controller();
        motors_output();
        break;
    }

    case TRANSITION_DONE:
        if (!tilt.motors_active) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            motors->output();
        }
        break;
    }
}

/*
  run multicopter rate controller
 */
void QuadPlane::run_rate_controller(void)
{
    attitude_control->set_throttle_mix_max();
    attitude_control->rate_controller_run();
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }

    if (plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }
    
    if (!in_vtol_mode()) {
        update_transition();
    } else {
        assisted_flight = false;
        
        // run low level rate controllers
        run_rate_controller();

        // output to motors
        motors_output();
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
        (plane.channel_throttle->get_control_in() > 10 ||
         plane.failsafe.ch3_failsafe ||
         plane.failsafe.ch3_counter>0)) {
        throttle_wait = false;
    }

    tiltrotor_update();
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DESIRED_SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::check_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DESIRED_THROTTLE_UNLIMITED) {
        return;
    }

    // if the users throttle is above zero then allow motors to run
    if (plane.channel_throttle->get_control_in() != 0) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.auto_throttle_mode && !plane.throttle_suppressed) {
        return;
    }

    // if our vertical velocity is greater than 1m/s then allow motors to run
    if (fabsf(inertial_nav.get_velocity_z()) > 100) {
        return;
    }

    // if we are more than 5m from home altitude then allow motors to run
    if (plane.relative_ground_altitude(plane.g.rangefinder_landing) > 5) {
        return;
    }

    // allow for takeoff
    if (plane.control_mode == AUTO && plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return;
    }
    
    // motors should be in the spin when armed state to warn user they could become active
    motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(void)
{
    if (!hal.util->get_soft_armed() || plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == QSTABILIZE) {
        // output is direct from run_esc_calibration()
        return;
    }

    // see if motors should be shut down
    check_throttle_suppression();
    
    motors->output();
    if (motors->armed()) {
        plane.DataFlash.Log_Write_Rate(plane.ahrs, *motors, *attitude_control, *pos_control);
        Log_Write_QControl_Tuning();
        uint32_t now = AP_HAL::millis();
        if (now - last_ctrl_log_ms > 100) {
            attitude_control->control_monitor_log();
        }
    }

    // remember when motors were last active for throttle suppression
    if (motors->get_throttle() > 0.01f || tilt.motors_active) {
        last_motors_active_ms = AP_HAL::millis();
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
    case QLAND:
        control_loiter();
        break;
    case QRTL:
        control_qrtl();
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

    AP_Notify::flags.esc_calibration = false;

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
    case QLAND:
        init_land();
        break;
    case QRTL:
        init_qrtl();
        break;
    default:
        break;
    }
    return true;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state)
{
    if (!available()) {
        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "VTOL not available");
        return false;
    }
    if (plane.control_mode != AUTO) {
        plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return false;
    }
    switch (state) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        return true;
        
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;

        return true;

    default:
        break;
    }

    plane.gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return false;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void)
{
    if (!enable) {
        return false;
    }
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
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
        return plane.auto_state.vtol_loiter;
    default:
        return false;
    }
}

/*
  are we in a VTOL mode?
 */
bool QuadPlane::in_vtol_mode(void)
{
    if (!enable) {
        return false;
    }
    return (plane.control_mode == QSTABILIZE ||
            plane.control_mode == QHOVER ||
            plane.control_mode == QLOITER ||
            plane.control_mode == QLAND ||
            plane.control_mode == QRTL ||
            ((plane.control_mode == GUIDED || plane.control_mode == AVOID_ADSB) && plane.auto_state.vtol_loiter) ||
            in_vtol_auto());
}


/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    setup_target_position();

    const Location &loc = plane.next_WP_loc;
    float ekfGndSpdLimit, ekfNavVelGainScaler;    
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    
    switch (poscontrol.state) {
    case QPOS_LAND_FINAL:
        /*
          for land-final we use the loiter controller
         */
    
        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                             plane.nav_pitch_cd,
                                                                             get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds(),
                                                                             smoothing_gain);
        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        plane.nav_pitch_cd = pos_control->get_pitch();
        break;

    case QPOS_POSITION1: {
        Vector2f diff_wp = location_diff(plane.current_loc, loc);
        float distance = diff_wp.length();

        if (poscontrol.speed_scale <= 0) {
            // initialise scaling so we start off targeting our
            // current linear speed towards the target. If this is
            // less than the wpnav speed then the wpnav speed is used
            // land_speed_scale is then used to linearly change
            // velocity as we approach the waypoint, aiming for zero
            // speed at the waypoint
            Vector2f groundspeed = ahrs.groundspeed_vector();
            float speed_towards_target = distance>1?(diff_wp.normalized() * groundspeed):0;
            // setup land_speed_scale so at current distance we
            // maintain speed towards target, and slow down as we
            // approach

            // max_speed will control how fast we will fly. It will always decrease
            poscontrol.max_speed = MAX(speed_towards_target, wp_nav->get_speed_xy() * 0.01);
            poscontrol.speed_scale = poscontrol.max_speed / MAX(distance, 1);

            // start with low integrator. The alt_hold controller will
            // add hover throttle to initial integrator. By starting
            // without it we end up with a smoother startup when
            // transitioning from fixed wing flight
            float aspeed;
            if (ahrs.airspeed_estimate(&aspeed) && aspeed > 6) {
                pid_accel_z.set_integrator((-motors->get_throttle_hover())*1000.0f);
            }
        }

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);

        /*
          calculate target velocity, not dropping it below 2m/s
         */
        const float final_speed = 2.0f;
        Vector2f target_speed_xy = diff_wp * poscontrol.speed_scale;
        float target_speed = target_speed_xy.length();
        if (distance < 1) {
            // prevent numerical error before switching to POSITION2
            target_speed_xy(0.1, 0.1);
        }
        if (target_speed < final_speed) {
            // until we enter the loiter we always aim for at least 2m/s
            target_speed_xy = target_speed_xy.normalized() * final_speed;
            poscontrol.max_speed = final_speed;
        } else if (target_speed > poscontrol.max_speed) {
            // we never speed up during landing approaches
            target_speed_xy = target_speed_xy.normalized() * poscontrol.max_speed;
        } else {
            poscontrol.max_speed = target_speed;
        }
        pos_control->set_desired_velocity_xy(target_speed_xy.x*100,
                                             target_speed_xy.y*100);
        
        pos_control->update_vel_controller_xyz(ekfNavVelGainScaler);

        const Vector3f& curr_pos = inertial_nav.get_position();
        pos_control->set_xy_target(curr_pos.x, curr_pos.y);

        pos_control->freeze_ff_xy();
        
        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        plane.nav_pitch_cd = pos_control->get_pitch();

        /*
          limit the pitch down with an expanding envelope. This
          prevents the velocity controller demanding nose down during
          the initial slowdown if the target velocity curve is higher
          than the actual velocity curve (for a high drag
          aircraft). Nose down will cause a lot of downforce on the
          wings which will draw a lot of current and also cause the
          aircraft to lose altitude rapidly.
         */
        float pitch_limit_cd = linear_interpolate(-300, plane.aparm.pitch_limit_min_cd,
                                                  plane.auto_state.wp_proportion, 0, 1);
        if (plane.nav_pitch_cd < pitch_limit_cd) {
            plane.nav_pitch_cd = pitch_limit_cd;
            // tell the pos controller we have limited the pitch to
            // stop integrator buildup
            pos_control->set_limit_accel_xy();
        }
        
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                             plane.nav_pitch_cd,
                                                                             desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds(),
                                                                             smoothing_gain);
        if (plane.auto_state.wp_proportion >= 1 ||
            plane.auto_state.wp_distance < 5) {
            poscontrol.state = QPOS_POSITION2;
            wp_nav->init_loiter_target();
            plane.gcs_send_text_fmt(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f",
                                    (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance);
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_DESCEND:
        /*
          for final land repositioning and descent we run the loiter controller
         */
        
        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);

        pos_control->set_xy_target(poscontrol.target.x, poscontrol.target.y);
        
        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = wp_nav->get_roll();
        plane.nav_pitch_cd = wp_nav->get_pitch();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                             plane.nav_pitch_cd,
                                                                             get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds(),
                                                                             smoothing_gain);
        break;

    case QPOS_LAND_COMPLETE:
        // nothing to do
        break;
    }

    // now height control
    switch (poscontrol.state) {
    case QPOS_POSITION1:
    case QPOS_POSITION2:
        if (plane.control_mode == QRTL) {
            plane.ahrs.get_position(plane.current_loc);
            float target_altitude = plane.next_WP_loc.alt;
            if (poscontrol.slow_descent) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = location_path_proportion(plane.current_loc, 
                                                                          plane.prev_WP_loc, plane.next_WP_loc);
                target_altitude = linear_interpolate(plane.prev_WP_loc.alt,
                                                     plane.next_WP_loc.alt,
                                                     plane.auto_state.wp_proportion,
                                                     0, 1);
            }
            pos_control->set_alt_target(target_altitude - plane.home.alt);
        } else {
            pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, false);
        }
        break;

    case QPOS_LAND_DESCEND: {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        pos_control->set_alt_target_from_climb_rate(-landing_descent_rate_cms(height_above_ground),
                                                    plane.G_Dt, true);
        break;
    }

    case QPOS_LAND_FINAL:
        pos_control->set_alt_target_from_climb_rate(-land_speed_cms, plane.G_Dt, true);
        break;
        
    case QPOS_LAND_COMPLETE:
        break;
    }
    
    pos_control->update_z_controller();
}


/*
  setup the target position based on plane.next_WP_loc
 */
void QuadPlane::setup_target_position(void)
{
    const Location &loc = plane.next_WP_loc;
    Location origin = inertial_nav.get_origin();
    Vector2f diff2d;

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    diff2d = location_diff(origin, loc);
    poscontrol.target.x = diff2d.x * 100;
    poscontrol.target.y = diff2d.y * 100;
    poscontrol.target.z = plane.next_WP_loc.alt - origin.alt;

    if (!locations_are_same(loc, last_auto_target) ||
        plane.next_WP_loc.alt != last_auto_target.alt ||
        millis() - last_loiter_ms > 500) {
        wp_nav->set_wp_destination(poscontrol.target);
        last_auto_target = loc;
    }
    last_loiter_ms = millis();
    
    // setup vertical speed and acceleration
    pos_control->set_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_accel_z(pilot_accel_z);
}

/*
  run takeoff controller to climb vertically
 */
void QuadPlane::takeoff_controller(void)
{
    /*
      for takeoff we need to use the loiter controller wpnav controller takes over the descent rate
      control
    */
    float ekfGndSpdLimit, ekfNavVelGainScaler;    
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    setup_target_position();
    
    // run loiter controller
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
    
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                         plane.nav_pitch_cd,
                                                                         get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds(),
                                                                         smoothing_gain);
    
    // nav roll and pitch are controller by position controller
    plane.nav_roll_cd = pos_control->get_roll();
    plane.nav_pitch_cd = pos_control->get_pitch();
    
    pos_control->set_alt_target_from_climb_rate(wp_nav->get_speed_up(), plane.G_Dt, true);
    pos_control->update_z_controller();
}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
void QuadPlane::waypoint_controller(void)
{
    setup_target_position();

    /*
      this is full copter control of auto flight
    */
    // run wpnav controller
    wp_nav->update_wpnav();
    
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(),
                                                       wp_nav->get_pitch(),
                                                       wp_nav->get_yaw(),
                                                       true, 4.0f);
    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();
    
    // climb based on altitude error
    pos_control->set_alt_target_from_climb_rate_ff(assist_climb_rate_cms(), plane.G_Dt, false);
    pos_control->update_z_controller();
}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(const Location &loc)
{
    if (!setup()) {
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    switch (plane.mission.get_current_nav_cmd().id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        takeoff_controller();
        break;
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
        vtol_position_controller();
        break;
    default:
        waypoint_controller();
        break;
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::control_qrtl(void)
{
    vtol_position_controller();
    if (poscontrol.state >= QPOS_POSITION2) {
        // change target altitude to home alt
        plane.next_WP_loc.alt = plane.home.alt;
        verify_vtol_land();
    } else {
        pos_control->set_alt_target(qrtl_alt*100UL);
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::init_qrtl(void)
{
    // use do_RTL() to setup next_WP_loc
    plane.do_RTL(plane.home.alt + qrtl_alt*100UL);
    plane.prev_WP_loc = plane.current_loc;
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
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
    attitude_control->get_rate_roll_pid().reset_I();
    attitude_control->get_rate_pitch_pid().reset_I();
    attitude_control->get_rate_yaw_pid().reset_I();
    pid_accel_z.reset_I();
    pi_vel_xy.reset_I();
    
    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.alt = plane.current_loc.alt;
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
    wp_nav->init_loiter_target();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    Location origin = inertial_nav.get_origin();
    Vector2f diff2d;
    Vector3f target;
    diff2d = location_diff(origin, plane.next_WP_loc);
    target.x = diff2d.x * 100;
    target.y = diff2d.y * 100;
    target.z = plane.next_WP_loc.alt - origin.alt;
    pos_control->set_alt_target(inertial_nav.get_altitude());
    
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
    plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
    pos_control->set_alt_target(inertial_nav.get_altitude());

    plane.complete_auto_takeoff();
    
    return true;
}

/*
  check if a landing is complete
 */
void QuadPlane::check_land_complete(void)
{
    if (poscontrol.state != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return;
    }
    uint32_t now = AP_HAL::millis();
    bool might_be_landed =  (landing_detect.lower_limit_start_ms != 0 &&
                             now - landing_detect.lower_limit_start_ms > 1000);
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return;
    }
    float height = inertial_nav.get_altitude()*0.01f;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height;
    }
    // we only consider the vehicle landed when the motors have been
    // at minimum for 5s and the vertical position estimate has not
    // changed by more than 20cm for 4s
    if (fabsf(height - landing_detect.vpos_start_m) > 0.2) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return;
    }
           
    if ((now - landing_detect.land_start_ms) < 4000 ||
        (now - landing_detect.lower_limit_start_ms) < 5000) {
        // not landed yet
        return;
    }
    landing_detect.land_start_ms = 0;
    // motors have been at zero for 5s, and we have had less than 0.3m
    // change in altitude for last 4s. We are landed.
    plane.disarm_motors();
    poscontrol.state = QPOS_LAND_COMPLETE;
    plane.gcs_send_text(MAV_SEVERITY_INFO,"Land complete");
    // reload target airspeed which could have been modified by the mission
    plane.g.airspeed_cruise_cm.load();
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }
    if (poscontrol.state == QPOS_POSITION2 &&
        plane.auto_state.wp_distance < 2) {
        poscontrol.state = QPOS_LAND_DESCEND;
        plane.gcs_send_text(MAV_SEVERITY_INFO,"Land descend started");
        plane.set_next_WP(plane.next_WP_loc);
    }

    if (should_relax()) {
        wp_nav->loiter_soften_for_landing();
    }

    // at land_final_alt begin final landing
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    if (poscontrol.state == QPOS_LAND_DESCEND && height_above_ground < land_final_alt) {
        poscontrol.state = QPOS_LAND_FINAL;
        pos_control->set_alt_target(inertial_nav.get_altitude());

        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0);
        }
        plane.gcs_send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    check_land_complete();
    return false;
}

// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{
    const Vector3f &desired_velocity = pos_control->get_desired_velocity();
    const Vector3f &accel_target = pos_control->get_accel_target();
    struct log_QControl_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_QTUN_MSG),
        time_us             : AP_HAL::micros64(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        desired_alt         : pos_control->get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : (int32_t)plane.barometer.get_altitude() * 100,
        desired_climb_rate  : (int16_t)pos_control->get_vel_target_z(),
        climb_rate          : (int16_t)inertial_nav.get_velocity_z(),
        dvx                 : desired_velocity.x*0.01f,
        dvy                 : desired_velocity.y*0.01f,
        dax                 : accel_target.x*0.01f,
        day                 : accel_target.y*0.01f,
    };
    plane.DataFlash.WriteBlock(&pkt, sizeof(pkt));
}


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
int8_t QuadPlane::forward_throttle_pct(void)
{
    /*
      in non-VTOL modes or modes without a velocity controller. We
      don't use it in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        vel_forward.gain <= 0 ||
        plane.control_mode == QSTABILIZE ||
        plane.control_mode == QHOVER) {
        return 0;
    }

    float deltat = (AP_HAL::millis() - vel_forward.last_ms) * 0.001f;
    if (deltat > 1 || deltat < 0) {
        vel_forward.integrator = 0;
        deltat = 0.1;
    }
    if (deltat < 0.1) {
        // run at 10Hz
        return vel_forward.last_pct;
    }
    vel_forward.last_ms = AP_HAL::millis();
    
    // work out the desired speed in forward direction
    const Vector3f &desired_velocity_cms = pos_control->get_desired_velocity();
    Vector3f vel_ned;
    if (!plane.ahrs.get_velocity_NED(vel_ned)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    Vector3f vel_error_body = ahrs.get_rotation_body_to_ned().transposed() * ((desired_velocity_cms*0.01f) - vel_ned);

    // find component of velocity error in fwd body frame direction
    float fwd_vel_error = vel_error_body * Vector3f(1,0,0);

    // scale forward velocity error by maximum airspeed
    fwd_vel_error /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error -= (wp_nav->get_speed_xy() * 0.01f) * plane.nav_pitch_cd / (float)plane.aparm.pitch_limit_max_cd;

    if (should_relax() && vel_ned.length() < 1) {
        // we may be landed
        fwd_vel_error = 0;
        vel_forward.integrator *= 0.95f;
    }
    
    // integrator as throttle percentage (-100 to 100)
    vel_forward.integrator += fwd_vel_error * deltat * vel_forward.gain * 100;

    // inhibit reverse throttle and allow petrol engines with min > 0
    int8_t fwd_throttle_min = (plane.aparm.throttle_min <= 0) ? 0 : plane.aparm.throttle_min;
    vel_forward.integrator = constrain_float(vel_forward.integrator, fwd_throttle_min, plane.aparm.throttle_max);
    
    // If we are below alt_cutoff then scale down the effect until it turns off at alt_cutoff and decay the integrator
    float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                   height_above_ground, alt_cutoff, alt_cutoff+2);
    if (vel_forward.last_pct == 0) {
        // if the percent is 0 then decay the integrator
        vel_forward.integrator *= 0.95f;
    }

    return vel_forward.last_pct;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    /*
      we only do weathervaning in modes where we are doing VTOL
      position control. We also don't do it if the pilot has given any
      yaw input in the last 3 seconds.
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        weathervane.gain <= 0 ||
        plane.control_mode == QSTABILIZE ||
        plane.control_mode == QHOVER) {
        weathervane.last_output = 0;
        return 0;
    }
    if (plane.channel_rudder->get_control_in() != 0) {
        weathervane.last_pilot_input_ms = AP_HAL::millis();
        weathervane.last_output = 0;
        return 0;
    }
    if (AP_HAL::millis() - weathervane.last_pilot_input_ms < 3000) {
        weathervane.last_output = 0;
        return 0;
    }

    float roll = wp_nav->get_roll() / 100.0f;
    if (fabsf(roll) < weathervane.min_roll) {
        weathervane.last_output = 0;
        return 0;        
    }
    if (roll > 0) {
        roll -= weathervane.min_roll;
    } else {
        roll += weathervane.min_roll;
    }
    
    float output = constrain_float((roll/45.0f) * weathervane.gain, -1, 1);
    if (should_relax()) {
        output = 0;
    }
    weathervane.last_output = 0.98f * weathervane.last_output + 0.02f * output;

    // scale over half of yaw_rate_max. This gives the pilot twice the
    // authority of the weathervane controller
    return weathervane.last_output * (yaw_rate_max/2) * 100;
}

/*
  start guided mode control
 */
void QuadPlane::guided_start(void)
{
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
    setup_target_position();
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{
    // run VTOL position controller
    vtol_position_controller();
}

void QuadPlane::afs_terminate(void)
{
    if (available()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
    }
}
