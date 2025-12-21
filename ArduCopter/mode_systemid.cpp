#include "Copter.h"
#include <AP_Math/control.h>

#if MODE_SYSTEMID_ENABLED

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust, 14:Measured Lateral Position, 15:Measured Longitudinal Position, 16:Measured Lateral Velocity, 17:Measured Longitudinal Velocity, 18:Input Lateral Velocity, 19:Input Longitudinal Velocity
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

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Aircraft must be flying");
        return false;
    }

    if (!is_poscontrol_axis_type()) {

        // System ID is being done on the attitude control loops

        // Can only switch into System ID Axes 1-13 with a flight mode that has manual throttle
        if (!copter.flightmode->has_manual_throttle()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires manual throttle");
            return false;
        }

#if FRAME_CONFIG == HELI_FRAME
        copter.input_manager.set_use_stab_col(true);
#endif

    } else {

        // System ID is being done on the position control loops

        // Can only switch into System ID Axes 14-19 from Loiter flight mode
        if (copter.flightmode->mode_number() != Mode::Number::LOITER) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires switch from Loiter");
            return false;
        }

        // set horizontal speed and acceleration limits
        pos_control->NE_set_max_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());
        pos_control->NE_set_correction_speed_accel_m(wp_nav->get_default_speed_NE_ms(), wp_nav->get_wp_acceleration_mss());

        // initialise the horizontal position controller
        if (!pos_control->NE_is_active()) {
            pos_control->NE_init_controller();
        }

        // set vertical speed and acceleration limits
        pos_control->D_set_max_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());
        pos_control->D_set_correction_speed_accel_m(wp_nav->get_default_speed_down_ms(), wp_nav->get_default_speed_up_ms(), wp_nav->get_accel_D_mss());

        // initialise the vertical position controller
        if (!pos_control->D_is_active()) {
            pos_control->D_init_controller();
        }
        target_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
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
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;
    float pilot_throttle_scaled = 0.0f;
    float target_climb_rate_ms = 0.0f;
    Vector2f input_vel_ne_ms;

    if (!is_poscontrol_axis_type()) {

        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

        // get pilot's desired yaw rate
        target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

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
            if (attitude_control->lean_angle_rad() > attitude_control->lean_angle_max_rad()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)degrees(attitude_control->lean_angle_rad()), (double)degrees(attitude_control->lean_angle_max_rad()));
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
                    target_roll_rad += radians(waveform_sample);
                    break;
                case AxisType::INPUT_PITCH:
                    target_pitch_rad += radians(waveform_sample);
                    break;
                case AxisType::INPUT_YAW:
                    target_yaw_rate_rads += radians(waveform_sample);
                    break;
                case AxisType::RECOVER_ROLL:
                    target_roll_rad += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_PITCH:
                    target_pitch_rad += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_YAW:
                    target_yaw_rate_rads += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RATE_ROLL:
                    attitude_control->rate_bf_roll_sysid_rads(radians(waveform_sample));
                    break;
                case AxisType::RATE_PITCH:
                    attitude_control->rate_bf_pitch_sysid_rads(radians(waveform_sample));
                    break;
                case AxisType::RATE_YAW:
                    attitude_control->rate_bf_yaw_sysid_rads(radians(waveform_sample));
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
                    disturb_state.y = waveform_sample;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_pos_NE_m(disturb_state);
                    break;
                case AxisType::DISTURB_POS_LONG:
                    disturb_state.x = waveform_sample;
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_pos_NE_m(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LAT:
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_vel_NE_ms(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LONG:
                    disturb_state.x = waveform_sample;
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);
                    pos_control->set_disturb_vel_NE_ms(disturb_state);
                    break;
                case AxisType::INPUT_VEL_LAT:
                    input_vel_ne_ms.x = 0.0f;
                    input_vel_ne_ms.y = waveform_sample;
                    input_vel_ne_ms.rotate(attitude_control->get_att_target_euler_rad().z);
                    break;
                case AxisType::INPUT_VEL_LONG:
                    input_vel_ne_ms.x = waveform_sample;
                    input_vel_ne_ms.y = 0.0f;
                    input_vel_ne_ms.rotate(attitude_control->get_att_target_euler_rad().z);
                    break;
            }
            break;
    }

    if (!is_poscontrol_axis_type()) {

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

        // output pilot's throttle
        attitude_control->set_throttle_out(pilot_throttle_scaled, !copter.is_tradheli(), g.throttle_filt);
        
    } else {

        // relax loiter target if we might be landed
        if (copter.ap.land_complete_maybe) {
            pos_control->NE_soften_for_landing();
        }

        Vector2f accel_ne_mss;
        target_pos_ne_m += input_vel_ne_ms.topostype() * G_Dt;
        if (is_positive(G_Dt)) {
            accel_ne_mss = (input_vel_ne_ms - input_vel_last_ne_ms) / G_Dt;
            input_vel_last_ne_ms = input_vel_ne_ms;
        }
        pos_control->set_pos_vel_accel_NE_m(target_pos_ne_m, input_vel_ne_ms, accel_ne_mss);

        // run pos controller
        pos_control->NE_update_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), target_yaw_rate_rads, false);

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

        // run the vertical position controller and set output throttle
        pos_control->D_update_controller();
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
    copter.Log_Write_Rate();
    copter.Log_Write_PIDS();

    if (is_poscontrol_axis_type()) {
        pos_control->write_log();
        copter.logger.Write_PID(LOG_PIDN_MSG, pos_control->NE_get_vel_pid().get_pid_info_x());
        copter.logger.Write_PID(LOG_PIDE_MSG, pos_control->NE_get_vel_pid().get_pid_info_y());

    }
}

bool ModeSystemId::is_poscontrol_axis_type() const
{
    bool ret = false;

    switch ((AxisType)axis.get()) {
        case AxisType::DISTURB_POS_LAT:
        case AxisType::DISTURB_POS_LONG:
        case AxisType::DISTURB_VEL_LAT:
        case AxisType::DISTURB_VEL_LONG:
        case AxisType::INPUT_VEL_LAT:
        case AxisType::INPUT_VEL_LONG:
            ret = true;
            break;
        default:
            break;
        }

    return ret;
}

#endif
