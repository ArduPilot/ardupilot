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

#if MODE_AUTOROTATE_ENABLED == ENABLED

#define AUTOROTATE_ENTRY_TIME          2.0f    // (s) number of seconds that the entry phase operates for
#define BAILOUT_MOTOR_RAMP_TIME        1.0f    // (s) time set on bailout ramp up timer for motors - See AC_MotorsHeli_Single
#define HEAD_SPEED_TARGET_RATIO        1.0f    // Normalised target main rotor head speed (unit: -)
#define TOUCHDOWN_TIME                 5.0f    // time in seconds after land complete flag until aircraft is disarmed 

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled
    if (!g2.arot.is_enable()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Not Enabled");
        return false;
    }

    // Check that interlock is disengaged
    if (motors->get_interlock()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Change Fail: Interlock Engaged");
        return false;
    }

    // init autorotation controllers object
    g2.arot.init(motors, copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270));

    // Retrieve rpm and start rpm sensor health checks
    _initial_rpm = g2.arot.get_rpm(true);

    // Display message
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

    // Set all intial flags to on
    _flags.entry_initial = true;
    _flags.ss_glide_initial = true;
    _flags.flare_initial = true;
    _flags.touch_down_initial = true;
    _flags.level_initial = true;
    _flags.break_initial = true;
    _flags.straight_ahead_initial = true;
    _flags.bail_out_initial = true;
    _msg_flags.bad_rpm = true;
    initial_energy_check = true;

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
    // Check if interlock becomes engaged
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = Autorotation_Phase::BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        // Aircraft is landed and no need to bail out
        set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
    }

    // Current time
    uint32_t now = millis(); //milliseconds

    // set dt in library
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    g2.arot.set_dt(last_loop_time_s);

    float alt = g2.arot.get_ground_distance();
    // have autorotation library update estimated radar altitude
    g2.arot.update_est_rangefinder_alt();
    if (alt < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        g2.arot._using_rfnd = true;
    } else {
        g2.arot._using_rfnd = false;
    }

    // Initialise internal variables
    float curr_vel_z = inertial_nav.get_velocity_z_up_cms();   // Current vertical descent

    // Update time to impact only if we are not in the touch down phase. This must be updated before the state machine.
    if (phase_switch != Autorotation_Phase::TOUCH_DOWN) {
        time_to_impact = g2.arot.get_time_to_ground();
    }


    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

    // initial check for total energy monitoring
    if (initial_energy_check) {
        hover_autorotation = (inertial_nav.get_speed_xy_cms() < 250.0f) && (g2.arot.get_ground_distance() < g2.arot.get_flare_alt());
        initial_energy_check = false;
    }

    //total energy check
    if (hover_autorotation) {
        if (_flags.entry_initial == false  && time_to_impact <= g2.arot.get_t_touchdown()) {
            phase_switch = Autorotation_Phase::TOUCH_DOWN;
        }
    } else {
        if ( _flags.ss_glide_initial == true  && _flags.flare_initial == true && _flags.touch_down_initial == true && g2.arot.get_est_alt() > g2.arot.get_flare_alt() ) {
            if ((now - _entry_time_start_ms)/1000.0f > AUTOROTATE_ENTRY_TIME )  {
                // Flight phase can be progressed to steady state glide
                phase_switch = Autorotation_Phase::SS_GLIDE;
            }
        } else if ( g2.arot.get_est_alt()<=g2.arot.get_flare_alt() && g2.arot.get_est_alt()>g2.arot.get_cushion_alt() && !g2.arot.is_flare_complete() ) {
            phase_switch = Autorotation_Phase::FLARE;
        } else if (time_to_impact <= g2.arot.get_t_touchdown() && _flags.flare_initial == false ) {
            phase_switch = Autorotation_Phase::TOUCH_DOWN;
        }
    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

    case Autorotation_Phase::ENTRY: {
        // Entry phase functions to be run only once
        if (_flags.entry_initial == true) {

            gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");

            // Set necessary variables for the entry phase in the autorotation obj
            g2.arot.init_entry();

            // Target head speed is set to rpm at initiation to prevent unwanted changes in attitude
            _target_head_speed = _initial_rpm/g2.arot.get_hs_set_point();

            // Prevent running the initial entry functions again
            _flags.entry_initial = false;

        }

        if (!hover_autorotation) {
            // Slowly change the target head speed until the target head speed matches the parameter defined value
            float norm_rpm = g2.arot.get_rpm()/g2.arot.get_hs_set_point();
            if (norm_rpm > HEAD_SPEED_TARGET_RATIO*1.005f  ||  norm_rpm < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay * last_loop_time_s;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }
            // Set target head speed in head speed controller
            g2.arot.set_target_head_speed(_target_head_speed);
            // Run airspeed/attitude controller
            g2.arot.update_forward_speed_controller();
            // Retrieve pitch target
            _pitch_target = g2.arot.get_pitch();
            // Update controllers
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(); //run head speed/ collective controller
        } else {
            _pitch_target = 0.0f;
            g2.arot.update_hover_autorotation_controller();
        }

        break;
    }

    case Autorotation_Phase::SS_GLIDE: {
        // Steady state glide functions to be run only once
        if (_flags.ss_glide_initial == true) {

            gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");

            // Ensure target hs is set to glide in case hs hasn't reached target for glide
            _target_head_speed = HEAD_SPEED_TARGET_RATIO;

            // Set necessary variables for the glide phase in the autorotation obj
            g2.arot.init_glide(_target_head_speed);

            // Prevent running the initial glide functions again
            _flags.ss_glide_initial = false;
        }

        // Run airspeed/attitude controller
        g2.arot.update_forward_speed_controller();

        //update sink rate derivative
        g2.arot.calc_sink_d_avg();

        //update flare altitude estimate
        g2.arot.update_flare_alt();

        // Retrieve pitch target
        _pitch_target = g2.arot.get_pitch();

        // Update head speed/ collective controller
        _flags.bad_rpm = g2.arot.update_hs_glide_controller();
        // Attitude controller is updated in navigation switch-case statements
        g2.arot.update_avg_acc_z();

        break;
    }

    case Autorotation_Phase::FLARE: {
        // Steady state glide functions to be run only once
        if (_flags.flare_initial == true) {
            gcs().send_text(MAV_SEVERITY_INFO, "Flare_Phase");

            // Ensure target head speed is set in head speed controller
            _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide incase hs has not reached target for glide

            g2.arot.init_flare(_target_head_speed);

            // Prevent running the initial flare functions again
            _flags.flare_initial = false;
        }
        // Run flare controller
        g2.arot.flare_controller();

        //update sink rate derivative
        g2.arot.calc_sink_d_avg();

        // Update head speed/ collective controller
        _flags.bad_rpm = g2.arot.update_hs_glide_controller();
        // Retrieve pitch target
        _pitch_target = g2.arot.get_pitch();

        //calc average acceleration on z axis for estimating flare effectiveness
        g2.arot.update_avg_acc_z();

        break;
    }
    case Autorotation_Phase::TOUCH_DOWN: {
        if (_flags.touch_down_initial == true) {
            gcs().send_text(MAV_SEVERITY_INFO, "Touchdown_Phase");
            // Prevent running the initial touchdown functions again
            _flags.touch_down_initial = false;
            _touchdown_time_ms = millis();
            g2.arot.init_touchdown();
        }
        g2.arot.touchdown_controller();
        _pitch_target = g2.arot.get_pitch();
        uint32_t disarm_delay_ms = 1000*g.disarm_delay;
        if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE && ((now -  _touchdown_time_ms) > disarm_delay_ms )) {
            copter.arming.disarm(AP_Arming::Method::LANDED);
        }

        break;
    }

    case Autorotation_Phase::BAIL_OUT: {
        if (_flags.bail_out_initial == true) {
            // Functions and settings to be done once are done here.
            gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");

            // Set bail out timer remaining equal to the parameter value, bailout time
            // cannot be less than the motor spool-up time: BAILOUT_MOTOR_RAMP_TIME.
            _bail_time = MAX(g2.arot.get_bail_time(),BAILOUT_MOTOR_RAMP_TIME+0.1f);

            // Set bail out start time
            _bail_time_start_ms = now;

            // Set initial target vertical speed
            _desired_v_z = curr_vel_z;

            // Initialise position and desired velocity
            if (!pos_control->is_active_z()) {
                pos_control->relax_z_controller(g2.arot.get_last_collective());
            }

            // Get pilot parameter limits
            const float pilot_spd_dn = -get_pilot_speed_dn();
            const float pilot_spd_up = g.pilot_speed_up;

            float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            pilot_des_v_z = constrain_float(pilot_des_v_z, pilot_spd_dn, pilot_spd_up);

            // Calculate target climb rate adjustment to transition from bail out descent speed to requested climb rate on stick.
            _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_bail_time - BAILOUT_MOTOR_RAMP_TIME); //accounting for 0.5s motor spool time

            // Calculate pitch target adjustment rate to return to level
            _target_pitch_adjust = _pitch_target/_bail_time;

            // set vertical speed and acceleration limits
            pos_control->set_max_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));
            pos_control->set_correction_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));

            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            _flags.bail_out_initial = false;
        }

        if ((now - _bail_time_start_ms)/1000.0f >= BAILOUT_MOTOR_RAMP_TIME) {
            // Update desired vertical speed and pitch target after the bailout motor ramp timer has completed
            _desired_v_z -= _target_climb_rate_adjust * last_loop_time_s;
            _pitch_target -= _target_pitch_adjust * last_loop_time_s;
        }
        // Set position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(_desired_v_z);

        // Update controllers
        pos_control->update_z_controller();

        if ((now - _bail_time_start_ms)/1000.0f >= _bail_time) {
            // Bail out timer complete.  Change flight mode. Do not revert back to auto. Prevent aircraft
            // from continuing mission and potentially flying further away after a power failure.
            if (copter.prev_control_mode == Mode::Number::AUTO) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::AUTOROTATION_BAILOUT);
            } else {
                set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
            }
        }

        break;
    }
    }

    float pilot_roll, pilot_pitch, pilot_yaw_rate;
    // Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch
    // is controlled by speed-height controller.
    get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    float target_roll;

    // Grab inertial velocity
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain scheduling for yaw
    float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
    pilot_yaw_rate = ((float)pilot_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g2.command_model_pilot.get_rate() / 45.0;

    roll_vel = constrain_float(roll_vel, -560.0f, 560.0f);
    pitch_vel = constrain_float(pitch_vel, -560.0f, 560.0f);

    // convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (pilot_roll / 0.8f);

    // roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -0.8f;
    target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);

    // Pitch target is calculated in autorotation phase switch above
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, _pitch_target, pilot_yaw_rate);

    // Output warning messaged if rpm signal is bad
    if (_flags.bad_rpm) {
        warning_message(1);
    }

} // End function run()

void ModeAutorotate::warning_message(uint8_t message_n)
{
    switch (message_n) {
    case 1: {
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
