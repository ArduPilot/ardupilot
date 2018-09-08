#include "Copter.h"

// FIXME? why are these static?
static bool land_with_gps;

static uint32_t land_start_time;
static bool land_pause;

// land_init - initialise land controller
bool Copter::ModeLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = copter.position_ok();
    if (land_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        loiter_nav->get_stopping_point_xy(stopping_point);
        loiter_nav->init_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control->set_max_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    
    land_start_time = millis();

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void Copter::ModeLand::run()
{
    if (land_with_gps) {
        gps_run();
    }else{
        nogps_run();
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::ModeLand::gps_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || motors->get_desired_spool_state() == AP_Motors::DESIRED_SPIN_WHEN_ARMED) {
        if (motors->get_spool_mode() == AP_Motors::SPIN_WHEN_ARMED || motors->get_spool_mode() == AP_Motors::SHUT_DOWN) {
            zero_throttle_and_relax_ac();
        } else {
            zero_throttle_and_hold_attitude();
        }  
        loiter_nav->init_target();
        pos_control->relax_alt_hold_controllers(0.0f);
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        if (motors->get_spool_mode() == AP_Motors::SPIN_WHEN_ARMED) {
            copter.init_disarm_motors();
        }
        return;
    }

    // if landed, spool down motors and disarm
    if (ap.land_complete) {
        zero_throttle_and_hold_attitude();
        loiter_nav->init_target();
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    // pause before beginning land descent
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }
    
    land_run_horizontal_control();
    land_run_vertical_control(land_pause);
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::ModeLand::nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    // *****CHECK LOGIC HERE.  I MADE IT SIMILAR TO GPS_RUN BUT KEPT ATTITUDE CONTROL STUFF *****
    if (!motors->armed() || !ap.auto_armed || motors->get_desired_spool_state() == AP_Motors::DESIRED_SPIN_WHEN_ARMED) {
        if (motors->get_spool_mode() == AP_Motors::SPIN_WHEN_ARMED || motors->get_spool_mode() == AP_Motors::SHUT_DOWN) {
            zero_throttle_and_relax_ac();
        } else {
            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            attitude_control->set_throttle_out(0,false,g.throttle_filt);
            zero_throttle_and_hold_attitude();
        }
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // disarm when the landing detector says we've landed and motors have spooled down
        if (ap.land_complete && (motors->get_spool_mode() == AP_Motors::SPIN_WHEN_ARMED)) {
            copter.init_disarm_motors();
        }
        return;
    }

    // if landed, spool down motors and disarm
    if (ap.land_complete) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
        zero_throttle_and_hold_attitude();
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // pause before beginning land descent
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }

    land_run_vertical_control(land_pause);
}

// do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void Copter::ModeLand::do_not_use_GPS()
{
    land_with_gps = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(mode_reason_t reason)
{
    set_mode(LAND, reason);
    land_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS()
{
    return (control_mode == LAND && land_with_gps);
}
