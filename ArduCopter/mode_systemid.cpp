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
    if (motors->armed() && copter.ap.land_complete && !copter.flightmode->has_manual_throttle()) {
        return false;
    }

#if FRAME_CONFIG == HELI_FRAME
    copter.input_manager.set_use_stab_col(true);
#endif

    att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

    copter.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);

    return true;
}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void ModeSystemId::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

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
    float pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());
#else
    float pilot_throttle_scaled = get_pilot_desired_throttle();
#endif

    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) &&
        (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_const_freq))) {
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    waveform_time += G_Dt;
    waveform_sample = waveform(waveform_time - SYSTEM_ID_DELAY);

    switch (systemid_state) {
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            break;
        case SystemIDModeState::SYSTEMID_STATE_TESTING:
            attitude_control->bf_feedforward(att_bf_feedforward);

            if (copter.ap.land_complete) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            if (attitude_control->lean_angle()*100 > attitude_control->lean_angle_max()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle(), (double)attitude_control->lean_angle_max());
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
            }
            break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    if (copter.is_tradheli()) {
        attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
    } else {
        attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
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
}

// init_test - initialises the test
float ModeSystemId::waveform(float time)
{
    float wMin = 2 * M_PI * frequency_start;
    float wMax = 2 * M_PI * frequency_stop;

    float window;
    float output;

    float B = logf(wMax / wMin);

    if (time <= 0.0f) {
        window = 0.0f;
    } else if (time <= time_fade_in) {
        window = 0.5 - 0.5 * cosf(M_PI * time / time_fade_in);
    } else if (time <= time_record - time_fade_out) {
        window = 1.0;
    } else if (time <= time_record) {
        window = 0.5 - 0.5 * cosf(M_PI * (time - (time_record - time_fade_out)) / time_fade_out + M_PI);
    } else {
        window = 0.0;
    }

    if (time <= 0.0f) {
        waveform_freq_rads = wMin;
        output = 0.0f;
    } else if (time <= time_const_freq) {
        waveform_freq_rads = wMin;
        output = window * waveform_magnitude * sinf(wMin * time - wMin * time_const_freq);
    } else if (time <= time_record) {
        waveform_freq_rads = wMin * expf(B * (time - time_const_freq) / (time_record - time_const_freq));
        output = window * waveform_magnitude * sinf((wMin * (time_record - time_const_freq) / B) * (expf(B * (time - time_const_freq) / (time_record - time_const_freq)) - 1));
    } else {
        waveform_freq_rads = wMax;
        output = 0.0f;
    }
    return output;
}

#endif
