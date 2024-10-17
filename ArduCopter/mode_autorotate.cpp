/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED

#define AUTOROTATE_ENTRY_TIME          2.0f    // (s) number of seconds that the entry phase operates for
#define HEAD_SPEED_TARGET_RATIO        1.0f    // Normalised target main rotor head speed (unit: -)
#define AUTOROTATION_MIN_MOVING_SPEED  100.0    // (cm/s) minimum speed used for "is moving" check

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled, make sure this is the first check as this is the most
    // important thing for users to fix if they are planning to use autorotation mode
    if (!g2.arot.is_enable()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Autorot Mode Not Enabled");
        return false;
    }

    // Must be armed to use mode, prevent triggering state machine on the ground
    if (!motors->armed() || copter.ap.land_complete || copter.ap.land_complete_maybe) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Autorot: Must be Armed and Flying");
        return false;
    }

    // Initialise controllers
    // This must be done before RPM value is fetched
    g2.arot.init_hs_controller();
    g2.arot.init_fwd_spd_controller();

    // Retrieve rpm and start rpm sensor health checks
    _initial_rpm = g2.arot.get_rpm(true);

    // Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

     // Set all intial flags to on
    _flags.entry_initial = true;
    _flags.ss_glide_initial = true;
    _flags.flare_initial = true;
    _flags.touch_down_initial = true;
    _flags.landed_initial = true;
    _flags.level_initial = true;
    _flags.break_initial = true;
    _flags.straight_ahead_initial = true;
    _msg_flags.bad_rpm = true;

    // Setting default starting switches
    phase_switch = Autorotation_Phase::ENTRY;

    // Set entry timer
    _entry_time_start_ms = millis();

    // The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_initial_rpm/g2.arot.get_hs_set_point()) - HEAD_SPEED_TARGET_RATIO) / AUTOROTATE_ENTRY_TIME;

    return true;
}



void ModeAutorotate::run()
{
    // Current time
    uint32_t now = millis(); //milliseconds

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     // Setting default phase switch positions
     nav_pos_switch = Navigation_Decision::USER_CONTROL_STABILISED;

    // Timer from entry phase to progress to glide phase
    if (phase_switch == Autorotation_Phase::ENTRY){
        if ((now - _entry_time_start_ms)/1000.0f > AUTOROTATE_ENTRY_TIME) {
            // Flight phase can be progressed to steady state glide
            phase_switch = Autorotation_Phase::SS_GLIDE;
        }
    }

    // Check if we believe we have landed. We need the landed state to zero all controls and make sure that the copter landing detector will trip
    bool speed_check = inertial_nav.get_velocity_z_up_cms() < AUTOROTATION_MIN_MOVING_SPEED &&
                     inertial_nav.get_speed_xy_cms() < AUTOROTATION_MIN_MOVING_SPEED;
    if (motors->get_below_land_min_coll() && AP::ins().is_still() && speed_check) {
        phase_switch = Autorotation_Phase::LANDED;
    }

    // Check if we are bailing out and need to re-set the spool state
    if (motors->autorotation_bailout()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY:
        {
            // Entry phase functions to be run only once
            if (_flags.entry_initial == true) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_entry_freq());

                // Target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                _target_head_speed = _initial_rpm/g2.arot.get_hs_set_point();

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Prevent running the initial entry functions again
                _flags.entry_initial = false;

            }

            // Slowly change the target head speed until the target head speed matches the parameter defined value
            if (g2.arot.get_rpm() > HEAD_SPEED_TARGET_RATIO*1.005f  ||  g2.arot.get_rpm() < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }

            // Set target head speed in head speed controller
            g2.arot.set_target_head_speed(_target_head_speed);

            // Run airspeed/attitude controller
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target
            _pitch_target = g2.arot.get_pitch();

            // Update controllers
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); //run head speed/ collective controller

            break;
        }

        case Autorotation_Phase::SS_GLIDE:
        {
            // Steady state glide functions to be run only once
            if (_flags.ss_glide_initial == true) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_glide_freq());

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Set target head speed in head speed controller
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide in case hs has not reached target for glide
                g2.arot.set_target_head_speed(_target_head_speed);

                // Prevent running the initial glide functions again
                _flags.ss_glide_initial = false;
            }

            // Run airspeed/attitude controller
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target 
            _pitch_target = g2.arot.get_pitch();

            // Update head speed/ collective controller
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); 
            // Attitude controller is updated in navigation switch-case statements

            break;
        }

        case Autorotation_Phase::FLARE:
        case Autorotation_Phase::TOUCH_DOWN:
        {
            break;
        }
        case Autorotation_Phase::LANDED:
        {
            // Entry phase functions to be run only once
            if (_flags.landed_initial == true) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Landed");
                #endif
                _flags.landed_initial = false;
            }
            // don't allow controller to continually ask for more pitch to build speed when we are on the ground, decay to zero smoothly
            _pitch_target *= 0.95;
            break;
        }
    }

    switch (nav_pos_switch) {

        case Navigation_Decision::USER_CONTROL_STABILISED:
        {
            // Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch 
            // is controlled by speed-height controller.
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            // Get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate();

            // Pitch target is calculated in autorotation phase switch above
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            break;
        }

        case Navigation_Decision::STRAIGHT_AHEAD:
        case Navigation_Decision::INTO_WIND:
        case Navigation_Decision::NEAREST_RALLY:
        {
            break;
        }
    }

    // Output warning messaged if rpm signal is bad
    if (_flags.bad_rpm) {
        warning_message(1);
    }

} // End function run()

void ModeAutorotate::warning_message(uint8_t message_n)
{
    switch (message_n) {
        case 1:
        {
            if (_msg_flags.bad_rpm) {
                // Bad rpm sensor health.
                gcs().send_text(MAV_SEVERITY_INFO, "Warning: Poor RPM Sensor Health");
                gcs().send_text(MAV_SEVERITY_INFO, "Action: Minimum Collective Applied");
                _msg_flags.bad_rpm = false;
            }
            break;
        }
    }
}

#endif
