#include "Copter.h"

// table of user settable parameters
const AP_Param::GroupInfo ModeLand::var_info[] = {

    // @Param: SPD_MS
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in m/s
    // @Units: m/s
    // @Range: 0.3 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPD_MS", 1, ModeLand, land_speed_ms, LAND_SPD_MS_DEFAULT),

    // @Param: SPD_HIGH_MS
    // @DisplayName: Land speed high
    // @Description: The descent speed for the first stage of landing in m/s. If this is zero then WP_SPD_DN is used
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("SPD_HIGH_MS", 2, ModeLand, land_speed_high_ms, 0),

    // @Param: ALT_LOW_M
    // @DisplayName: Land alt low
    // @Description: Altitude during Landing at which vehicle slows to LAND_SPD_MS
    // @Units: m
    // @Range: 1 100
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ALT_LOW_M", 3, ModeLand, land_alt_low_m, 10),

    AP_GROUPEND
};

// constructor
ModeLand::ModeLand() : Mode()
{
    // load parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

// convert parameters
void ModeLand::convert_params()
{
    // PARAMETER_CONVERSION - Added: Jan 2026

    // return immediately if parameter conversion has already been performed
    if (land_speed_ms.configured() || land_speed_high_ms.configured() || land_alt_low_m.configured()) {
        return;
    }

    static const AP_Param::ConversionInfo conversion_info[] = {
        { Parameters::k_param_land_speed_cms, 0, AP_PARAM_INT16, "LAND_SPD_MS" },     // LAND_SPEED moved to LAND_SPD_MS
        { Parameters::k_param_land_speed_high_cms, 0, AP_PARAM_INT16, "LAND_SPD_HIGH_MS" },   // LAND_SPEED_HIGH moved to LAND_SPD_HIGH_MS
        { Parameters::k_param_g2, 25, AP_PARAM_INT16, "LAND_ALT_LOW_M" },  // LAND_ALT_LOW moved to LAND_ALT_LOW_M
    };
    AP_Param::convert_old_parameters_scaled(conversion_info, ARRAY_SIZE(conversion_info), 0.01, 0);
}

// land_init - initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();

    // set horizontal speed and acceleration limits
    pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
    pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

    // initialise the horizontal position controller
    if (control_position && !pos_control->NE_is_active()) {
        pos_control->NE_init_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    land_start_time = millis();
    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_PRECLAND_ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    if (control_position) {
        gps_run();
    } else {
        nogps_run();
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeLand::gps_run()
{
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // run normal landing or precision landing (if enabled)
        land_run_normal_or_precland(land_pause);
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{
    float target_roll_rad = 0.0f, target_pitch_rad = 0.0f;

    // process pilot inputs
    if (rc().has_valid_input()) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
        }
    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_vertical_control(land_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, auto_yaw.get_heading().yaw_rate_rads);
}

// do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void ModeLand::do_not_use_GPS()
{
    control_position = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    mode_land.set_land_pause(true);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}
