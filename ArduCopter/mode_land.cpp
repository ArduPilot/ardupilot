#include "Copter.h"

// land_init - initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();
    if (control_position) {
        // set target to stopping point
        Vector2f stopping_point;
        loiter_nav->get_stopping_point_xy(stopping_point);
        loiter_nav->init_target(stopping_point);
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    land_start_time = millis();
    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
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
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_horizontal_control();
        land_run_vertical_control(land_pause);
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
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
