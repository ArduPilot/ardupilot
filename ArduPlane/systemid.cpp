#include "systemid.h"

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/control.h>
#include "Plane.h"

/*
  handle systemid via an auxiliary switch
 */

const AP_Param::GroupInfo AP_SystemID::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:VTOL Input Roll Angle, 2:VTOL Input Pitch Angle, 3:VTOL Input Yaw Angle, 4:VTOL Recovery Roll Angle, 5:VTOL Recovery Pitch Angle, 6:VTOL Recovery Yaw Angle, 7:VTOL Rate Roll, 8:VTOL Rate Pitch, 9:VTOL Rate Yaw, 10:VTOL Mixer Roll, 11:VTOL Mixer Pitch, 12:VTOL Mixer Yaw, 13:VTOL Mixer Thrust, 20:FW Input Roll Angle, 21:FW Input Pitch Angle, 22:FW Mixer Roll, 23:FW Mixer Pitch
    AP_GROUPINFO_FLAGS("_AXIS", 1, AP_SystemID, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, AP_SystemID, waveform_magnitude, 5),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, AP_SystemID, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, AP_SystemID, frequency_stop, 15),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, AP_SystemID, time_fade_in, 5),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, AP_SystemID, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, AP_SystemID, time_fade_out, 1),

    // @Param: _XY_CTRL_MUL
    // @DisplayName: System identification XY control multiplier
    // @Description: A multiplier for the XY velocity and position controller when using systemID in VTOL modes that do horizontal position and velocity control
    // @Range: 0.05 1.0
    // @User: Standard
    AP_GROUPINFO("_XY_CTRL_MUL", 8, AP_SystemID, xy_control_mul, 0.1),
    
    AP_GROUPEND
};

AP_SystemID::AP_SystemID(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

/*
  start systemid
 */
void AP_SystemID::start()
{
    start_axis = axis;

    switch (start_axis) {
        case AxisType::NONE:
            // check if enabled
            gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: No axis selected");
            return;
        case AxisType::INPUT_ROLL:
        case AxisType::INPUT_PITCH:
        case AxisType::INPUT_YAW:
        case AxisType::RECOVER_ROLL:
        case AxisType::RECOVER_PITCH:
        case AxisType::RECOVER_YAW:
        case AxisType::RATE_ROLL:
        case AxisType::RATE_PITCH:
        case AxisType::RATE_YAW:
        case AxisType::MIX_ROLL:
        case AxisType::MIX_PITCH:
        case AxisType::MIX_YAW:
        case AxisType::MIX_THROTTLE:
            // Exits if the current flight mode or phase does not support system ID axis.
            if (!plane.control_mode->supports_vtol_systemid()) {
#if HAL_QUADPLANE_ENABLED
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight mode");
#else
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported in Plane");
#endif
                return;
            }
            break;
        case AxisType::FW_INPUT_ROLL:
        case AxisType::FW_INPUT_PITCH:
        case AxisType::FW_MIX_ROLL:
        case AxisType::FW_MIX_PITCH:
            // Exits if the currently flight mode or phase does not support system ID axis.
            if (!plane.control_mode->supports_fw_systemid()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight mode");
                return;
            }
            if (!plane.control_mode->allow_fw_systemid()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Axis not supported for this flight phase");
                return;
            }
            break;
    }

    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: must be armed");
        return;
    }

    attitude_offset_deg.zero();
    throttle_offset = 0;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.available()) {
        restore.att_bf_feedforward = plane.quadplane.attitude_control->get_bf_feedforward();
    }
#endif

    waveform_time = 0;
    time_const_freq = 2.0 / frequency_start; // Two full cycles at the starting frequency

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("SIDS", "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout",
                                "s--ssssss", "F--------", "QBfffffff",
                                AP_HAL::micros64(),
                                uint8_t(start_axis),
                                waveform_magnitude.get(),
                                frequency_start.get(),
                                frequency_stop.get(),
                                time_fade_in.get(),
                                time_const_freq,
                                time_record.get(),
                                time_fade_out.get());
#endif // HAL_LOGGING_ENABLED

    running = true;
}

/*
  stop systemid
 */
void AP_SystemID::stop()
{
    if (running) {
        running = false;
        attitude_offset_deg.zero();
        throttle_offset = 0;

#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.available()) {
            auto *attitude_control = plane.quadplane.attitude_control;
            attitude_control->bf_feedforward(restore.att_bf_feedforward);
            attitude_control->rate_bf_roll_sysid_rads(0);
            attitude_control->rate_bf_pitch_sysid_rads(0);
            attitude_control->rate_bf_yaw_sysid_rads(0);
            plane.quadplane.pos_control->NE_set_control_scale_factor(1);

            // re-initialise the XY controller so we take current position as target
            plane.quadplane.pos_control->NE_init_controller();
        }
#endif
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID stopped");
    }
}

/*
  update systemid - needs to be called at main loop rate
 */
void AP_SystemID::vtol_update()
{
#if HAL_QUADPLANE_ENABLED
    if (!running) {
        return;
    }
    if (chirp_input.completed()) {
        stop();
        return;
    }

    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

    waveform_time += last_loop_time_s;
    waveform_sample = chirp_input.update(waveform_time, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    auto *attitude_control = plane.quadplane.attitude_control;

    switch (start_axis) {
        case AxisType::NONE:
            // not possible, see start()
            break;
        case AxisType::INPUT_ROLL:
            attitude_offset_deg.x = waveform_sample;
            break;
        case AxisType::INPUT_PITCH:
            attitude_offset_deg.y = waveform_sample;
            break;
        case AxisType::INPUT_YAW:
            attitude_offset_deg.z = waveform_sample;
            break;
        case AxisType::RECOVER_ROLL:
            attitude_offset_deg.x = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_PITCH:
            attitude_offset_deg.y = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_YAW:
            attitude_offset_deg.z = waveform_sample;
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
            throttle_offset = waveform_sample;
            break;
        default:
            break;
    }

    // reduce control in NE axis when in position controlled modes
    plane.quadplane.pos_control->NE_set_control_scale_factor(xy_control_mul);

    if (log_subsample <= 0) {
        log_data();
        // log attitude controller at the same rate
        plane.quadplane.Log_Write_AttRate();

        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;

#endif
}

// Return true if a fixed wing system ID is currently running
bool AP_SystemID::is_running_fw() const
{
    if (!is_running()) {
        return false;
    }

    switch (start_axis) {
        case AxisType::NONE:
        case AxisType::INPUT_ROLL:
        case AxisType::INPUT_PITCH:
        case AxisType::INPUT_YAW:
        case AxisType::RECOVER_ROLL:
        case AxisType::RECOVER_PITCH:
        case AxisType::RECOVER_YAW:
        case AxisType::RATE_ROLL:
        case AxisType::RATE_PITCH:
        case AxisType::RATE_YAW:
        case AxisType::MIX_ROLL:
        case AxisType::MIX_PITCH:
        case AxisType::MIX_YAW:
        case AxisType::MIX_THROTTLE:
            break;

        case AxisType::FW_INPUT_ROLL:
        case AxisType::FW_INPUT_PITCH:
        case AxisType::FW_MIX_ROLL:
        case AxisType::FW_MIX_PITCH:
            return true;
    }

    return false;
}

/*
  update systemid - needs to be called at main loop rate
 */
void AP_SystemID::fw_update()
{
    if (!plane.control_mode->allow_fw_systemid() || chirp_input.completed()) {
        // Control mode change means chirp should be stopped, or
        // Chirp is complete
        stop();
        return;
    }

    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

    waveform_time += last_loop_time_s;
    waveform_sample = chirp_input.update(waveform_time, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    switch (start_axis) {
        case AxisType::NONE:
            // not possible, see start()
            break;
        case AxisType::FW_INPUT_ROLL:
            plane.nav_roll_cd += waveform_sample * 100.0f;
            break;
        case AxisType::FW_INPUT_PITCH:
            plane.nav_pitch_cd += waveform_sample * 100.0f;
            break;
        case AxisType::FW_MIX_ROLL:
            output_offset.x = waveform_sample;
            break;
        case AxisType::FW_MIX_PITCH:
            output_offset.y = waveform_sample;
            break;
        default:
            break;
    }

    if (log_subsample <= 0) {
        log_data();
        log_plane_data();

        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

// log system id
void AP_SystemID::log_data() const
{
#if HAL_LOGGING_ENABLED
    Vector3f delta_angle;
    float delta_angle_dt;
    plane.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    plane.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        const float dt_ang_inv = 1.0 / delta_angle_dt;
        const float dt_vel_inv = 1.0 / delta_velocity_dt;
        AP::logger().WriteStreaming("SIDD", "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az",
                                    "ss-zkkkooo", "F---------", "Qfffffffff",
                                    AP_HAL::micros64(),
                                    waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI),
                                    degrees(delta_angle.x * dt_ang_inv),
                                    degrees(delta_angle.y * dt_ang_inv),
                                    degrees(delta_angle.z * dt_ang_inv),
                                    delta_velocity.x * dt_vel_inv,
                                    delta_velocity.y * dt_vel_inv,
                                    delta_velocity.z * dt_vel_inv);
    }
#endif // HAL_LOGGING_ENABLED
}

// @LoggerMessage: SIDP
// @Description: System ID data for Plane
// @Field: TimeUS: Time since system startup
// @Field: DRll: Desired Roll Angle
// @Field: Rll: Roll Angle
// @Field: DPit: Desired Pitch Angle
// @Field: Pit: Pitch Angle
// @Field: rdes: Desired Roll Rate
// @Field: r: Measured Roll Rate
// @Field: pdes: Desired Pitch Rate
// @Field: p: Measured Pitch Rate
// @Field: Aile: Aileron
// @Field: Elev: Elevator
// @Field: aspd: Speed_Scalar
// @Field: eastas: EAS2TAS

void AP_SystemID::log_plane_data() const
{
#if HAL_LOGGING_ENABLED
   // int16_t pitch = plane.ahrs.pitch_sensor - plane.g.pitch_trim * 100;
    float speed_scaler = plane.get_speed_scaler();
    const auto &pitch_pid_info = plane.pitchController.get_pid_info();
    const auto &roll_pid_info = plane.rollController.get_pid_info();

    int16_t demanded_pitch = plane.nav_pitch_cd + int32_t(plane.g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * plane.g.kff_throttle_to_pitch;
    AP::logger().WriteStreaming("SIDP", "TimeUS,DRll,Rll,DPit,Pit,rdes,r,pdes,p,Aile,Elev,aspd,eastas",
                                "soooooooooooo", "F------------", "Qffffffffffff",
                                AP_HAL::micros64(),
                                plane.nav_roll_cd * 0.01f,
                                plane.ahrs.roll_sensor * 0.01f,
                                demanded_pitch * 0.01f,
                                plane.ahrs.pitch_sensor * 0.01f,
                                roll_pid_info.target,
                                degrees(plane.ahrs.get_gyro().x),
                                pitch_pid_info.target,
                                degrees(plane.ahrs.get_gyro().y),
                                SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * 0.01f,
                                SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * 0.01f,
                                speed_scaler,
                                plane.ahrs.get_EAS2TAS());

#endif // HAL_LOGGING_ENABLED
}
#endif // AP_PLANE_SYSTEMID_ENABLED

