#include "systemid.h"

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/control.h>
#include "Plane.h"

/*
  handle systemid via an auxillary switch
 */

const AP_Param::GroupInfo AP_SystemID::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust
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

    // check if enabled
    if (start_axis == AxisType::NONE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: No axis selected");
        return;
    }
    if (!plane.control_mode->supports_systemid()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Not supported in mode %s", plane.control_mode->name());
        return;
    }
    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: must be armed");
        return;
    }
    if (!plane.quadplane.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: only for quadplane");
        return;
    }

    attitude_offset_deg.zero();
    throttle_offset = 0;

    restore.att_bf_feedforward = plane.quadplane.attitude_control->get_bf_feedforward();

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

        auto *attitude_control = plane.quadplane.attitude_control;
        attitude_control->bf_feedforward(restore.att_bf_feedforward);
        attitude_control->rate_bf_roll_sysid(0);
        attitude_control->rate_bf_pitch_sysid(0);
        attitude_control->rate_bf_yaw_sysid(0);
        plane.quadplane.pos_control->set_xy_control_scale_factor(1);

        // re-initialise the XY controller so we take current position as target
        plane.quadplane.pos_control->init_xy_controller();

        gcs().send_text(MAV_SEVERITY_INFO, "SystemID stopped");
    }
}

/*
  update systemid - needs to be called at main loop rate
 */
void AP_SystemID::update()
{
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
            throttle_offset += waveform_sample;
            break;
    }

    // reduce control in XY axis when in position controlled modes
    plane.quadplane.pos_control->set_xy_control_scale_factor(xy_control_mul);

    log_data();
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

        // log attitude controller at the same rate
        plane.Log_Write_Attitude(true);
    }
#endif // HAL_LOGGING_ENABLED
}

#endif // AP_PLANE_SYSTEMID_ENABLED

