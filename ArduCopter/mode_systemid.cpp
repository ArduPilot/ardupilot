#include "Copter.h"

#if MODE_SYSTEMID_ENABLED == ENABLED

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeSystemId, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, ModeSystemId, waveform_magnitude, 15),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, ModeSystemId, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeSystemId, frequency_stop, 40),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, ModeSystemId, time_fade_in, 15),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, ModeSystemId, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeSystemId, time_fade_out, 2),

    AP_GROUPEND
};

ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // time in seconds waited after system id mode change for frequency sweep injection

// systemId_init - initialise systemId controller
bool ModeSystemId::init(bool ignore_checks)
{
    // check if enabled
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && copter.ap.land_complete && (!copter.flightmode->has_manual_throttle() || copter.flightmode->mode_number() == Mode::Number::LOITER)) {
        return false;
    }

    if ((AxisType)axis.get() != AxisType::DISTURB_POS_LAT && (AxisType)axis.get() != AxisType::DISTURB_POS_LONG 
         && (AxisType)axis.get() != AxisType::DISTURB_VEL_LAT && (AxisType)axis.get() != AxisType::DISTURB_VEL_LONG
         && (AxisType)axis.get() != AxisType::INPUT_LOITER_LAT && (AxisType)axis.get() != AxisType::INPUT_LOITER_LONG) {

#if FRAME_CONFIG == HELI_FRAME
        copter.input_manager.set_use_stab_col(true);
#endif

    } else {

        if (!copter.failsafe.radio) {
            float target_roll, target_pitch;
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

            // process pilot's roll and pitch input
            loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        } else {
            // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
            loiter_nav->clear_pilot_desired_acceleration();
        }
        loiter_nav->init_target();

        // initialise the vertical position controller
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

// #if AC_PRECLAND_ENABLED
//        _precision_loiter_active = false;
// #endif
    }

    att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    copter.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);
#endif

    return true;
}

// systemId_exit - clean up systemId controller before exiting
void ModeSystemId::exit()
{
    // reset the feedforward enabled parameter to the initialized state
    attitude_control->bf_feedforward(att_bf_feedforward);
}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void ModeSystemId::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float pilot_throttle_scaled = 0.0f;
    float target_climb_rate = 0.0f;
    Vector2f input_vel;
    Vector2f pilot_vel;

    if ((AxisType)axis.get() != AxisType::DISTURB_POS_LAT && (AxisType)axis.get() != AxisType::DISTURB_POS_LONG 
         && (AxisType)axis.get() != AxisType::DISTURB_VEL_LAT && (AxisType)axis.get() != AxisType::DISTURB_VEL_LONG
         && (AxisType)axis.get() != AxisType::INPUT_LOITER_LAT && (AxisType)axis.get() != AxisType::INPUT_LOITER_LONG) {

        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        if (!motors->armed()) {
            // Motors should be Stopped
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        // Tradheli doesn't set spool state to ground idle when throttle stick is zero.  Ground idle only set when
        // motor interlock is disabled.
        } else if (copter.ap.throttle_zero && !copter.is_tradheli()) {
            // Attempting to Land
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        switch (motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
            // Motors Stopped
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms();
            break;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // Landed
            // Tradheli initializes targets when going from disarmed to armed state. 
            // init_targets_on_arming is always set true for multicopter.
            if (motors->init_targets_on_arming()) {
                attitude_control->reset_yaw_target_and_rate();
                attitude_control->reset_rate_controller_I_terms_smoothly();
            }
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            // clear landing flag above zero throttle
            if (!motors->limit.throttle_lower) {
                set_land_complete(false);
            }
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // do nothing
            break;
        }

        // get pilot's desired throttle
#if FRAME_CONFIG == HELI_FRAME
        pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());
#else
        pilot_throttle_scaled = get_pilot_desired_throttle();
#endif

    } else {
        // set xy speed and acceleration limits
        pos_control->set_max_speed_accel_xy(500.0f, 250.0f);
        pos_control->set_correction_speed_accel_xy(500.0f, 250.0f);

        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

        // process pilot inputs unless we are in radio failsafe
        // this was split into two parts to allow pilot input to accomplish closed loop sweeps
        if (!copter.failsafe.radio) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            pilot_vel = get_pilot_desired_velocity(250.0f);

        }

    }

    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) &&
        (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_const_freq))) {
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    waveform_time += G_Dt;
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();
    Vector2f disturb_state;
    switch (systemid_state) {
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            attitude_control->bf_feedforward(att_bf_feedforward);
            break;
        case SystemIDModeState::SYSTEMID_STATE_TESTING:

            if (copter.ap.land_complete) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            if (attitude_control->lean_angle_deg()*100 > attitude_control->lean_angle_max_cd()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle_deg(), (double)attitude_control->lean_angle_max_cd());
                break;
            }
            if (waveform_time > SYSTEM_ID_DELAY + time_fade_in + time_const_freq + time_record + time_fade_out) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            switch ((AxisType)axis.get()) {
                case AxisType::NONE:
                    systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;
                case AxisType::INPUT_ROLL:
                    target_roll += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_PITCH:
                    target_pitch += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_YAW:
                    target_yaw_rate += waveform_sample*100.0f;
                    break;
                case AxisType::RECOVER_ROLL:
                    target_roll += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_PITCH:
                    target_pitch += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_YAW:
                    target_yaw_rate += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RATE_ROLL:
                    attitude_control->rate_bf_roll_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_PITCH:
                    attitude_control->rate_bf_pitch_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_YAW:
                    attitude_control->rate_bf_yaw_sysid(radians(waveform_sample));
                    break;
                case AxisType::MIX_ROLL:
                    attitude_control->actuator_roll_sysid(waveform_sample);
                    break;
                case AxisType::MIX_PITCH:
                    attitude_control->actuator_pitch_sysid(waveform_sample);
                    break;
                case AxisType::MIX_YAW:
                    attitude_control->actuator_yaw_sysid(waveform_sample);
                    break;
                case AxisType::MIX_THROTTLE:
                    pilot_throttle_scaled += waveform_sample;
                    break;
                case AxisType::DISTURB_POS_LAT:
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;
                    copter.rotate_target_body_frame_to_NE(disturb_state.x, disturb_state.y);
                    pos_control->set_disturb_pos_cm(disturb_state);
                    break;
                case AxisType::DISTURB_POS_LONG:
                    disturb_state.x = waveform_sample * 100.0f;
                    disturb_state.y = 0.0f;
                    copter.rotate_target_body_frame_to_NE(disturb_state.x, disturb_state.y);
                    pos_control->set_disturb_pos_cm(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LAT:
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;
                    copter.rotate_target_body_frame_to_NE(disturb_state.x, disturb_state.y);
                    pos_control->set_disturb_vel_cms(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LONG:
                    disturb_state.x = waveform_sample * 100.0f;
                    disturb_state.y = 0.0f;
                    copter.rotate_target_body_frame_to_NE(disturb_state.x, disturb_state.y);
                    pos_control->set_disturb_vel_cms(disturb_state);
                    break;
                case AxisType::INPUT_LOITER_LAT:
                    input_vel.x = 0.0f;
                    input_vel.y = waveform_sample * 100.0f;
                    copter.rotate_target_body_frame_to_NE(input_vel.x, input_vel.y);
                    break;
                case AxisType::INPUT_LOITER_LONG:
                    input_vel.x = waveform_sample * 100.0f;
                    input_vel.y = 0.0f;
                    copter.rotate_target_body_frame_to_NE(input_vel.x, input_vel.y);
                    break;
            }
            break;
    }

    if ((AxisType)axis.get() != AxisType::DISTURB_POS_LAT && (AxisType)axis.get() != AxisType::DISTURB_POS_LONG 
         && (AxisType)axis.get() != AxisType::DISTURB_VEL_LAT && (AxisType)axis.get() != AxisType::DISTURB_VEL_LONG
         && (AxisType)axis.get() != AxisType::INPUT_LOITER_LAT && (AxisType)axis.get() != AxisType::INPUT_LOITER_LONG) {

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_throttle_scaled, !copter.is_tradheli(), g.throttle_filt);
        
    } else {

        // process pilot inputs unless we are in radio failsafe
        // this was split into two parts to allow pilot input to accomplish closed loop sweeps
        if (!copter.failsafe.radio) {

            // Set pilot's roll and pitch input to zero
            loiter_nav->set_pilot_desired_acceleration(0.0f, 0.0f);

            // get pilot's desired yaw rate
            target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

            // get pilot desired climb rate
            target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        } else {
            // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
            loiter_nav->clear_pilot_desired_acceleration();
        }

        // relax loiter target if we might be landed
        if (copter.ap.land_complete_maybe) {
            pos_control->soften_for_landing_xy();
        }

        // Loiter State Machine Determination
        AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

        // Loiter State Machine
        switch (althold_state) {

        case AltHold_MotorStopped:
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->reset_yaw_target_and_rate();
            pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
            pos_control->init_xy_controller();
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            break;

        case AltHold_Landed_Ground_Idle:
            attitude_control->reset_yaw_target_and_rate();
            FALLTHROUGH;

        case AltHold_Landed_Pre_Takeoff:
            attitude_control->reset_rate_controller_I_terms_smoothly();
            pos_control->init_xy_controller();
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
            break;

        case AltHold_Takeoff:
            // initiate take-off
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            }

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // set position controller targets adjusted for pilot input
            takeoff.do_pilot_takeoff(target_climb_rate);

            // run loiter controller
            pos_control->update_xy_controller();

            // call attitude controller
            attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate, false);
            break;

        case AltHold_Flying:
            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            input_vel += pilot_vel;
            Vector2f accel = Vector2f((input_vel.x - target_vel.x) / G_Dt, (input_vel.y - target_vel.y) / G_Dt);

            target_vel = input_vel;
            pos_control->input_vel_accel_xy(target_vel, accel);
            pos_control->update_xy_controller();

            // call attitude controller
            attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate, false);

            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

            // update the vertical offset based on the surface measurement
            copter.surface_tracking.update_surface_offset();

            // Send the commanded climb rate to the position controller
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
        }

        // run the vertical position controller and set output throttle
        pos_control->update_z_controller();

    }

    if (log_subsample <= 0) {
        log_data();
        if (copter.should_log(MASK_LOG_ATTITUDE_FAST) && copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// log system id and attitude
void ModeSystemId::log_data() const
{
    Vector3f delta_angle;
    float delta_angle_dt;
    copter.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    copter.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        copter.Log_Write_SysID_Data(waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI), degrees(delta_angle.x / delta_angle_dt), degrees(delta_angle.y / delta_angle_dt), degrees(delta_angle.z / delta_angle_dt), delta_velocity.x / delta_velocity_dt, delta_velocity.y / delta_velocity_dt, delta_velocity.z / delta_velocity_dt);
    }

    // Full rate logging of attitude, rate and pid loops
    copter.Log_Write_Attitude();
    copter.Log_Write_PIDS();

    if ((AxisType)axis.get() == AxisType::DISTURB_POS_LAT || (AxisType)axis.get() == AxisType::DISTURB_POS_LONG 
         || (AxisType)axis.get() == AxisType::DISTURB_VEL_LAT || (AxisType)axis.get() == AxisType::DISTURB_VEL_LONG
         || (AxisType)axis.get() == AxisType::INPUT_LOITER_LAT || (AxisType)axis.get() == AxisType::INPUT_LOITER_LONG) {
        pos_control->write_log();
        copter.logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x());
        copter.logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y());

    }
}

#endif
