#include "Copter.h"

/*
 * Function to update various parameters in flight using the TRANSMITTER_TUNING channel knob
 * This should not be confused with the AutoTune feature which can be found in control_autotune.cpp
 */

// tuning - updates parameters based on the TRANSMITTER_TUNING channel knob's position
//  should be called at 3.3hz
void Copter::tuning()
{
    if (rc_tuning == nullptr) {
        // tuning channel is not set - don't know where to take input value from
        return;
    }

    if (g.rc_tuning_param <= 0) {
        // no parameter set for tuning
        return;
    }

    // check endpoints are not both zero:
    if (is_zero(g2.tuning_min.get()) && is_zero(g2.tuning_max.get())) {
        // both endpoints are zero, there is no input range to tune across
        return;
    }

    float norm_in;
    if (!tuning_ch->norm_input_ignore_trim(norm_in)) {
        // unable to get input for this channel (eg. in RC failsafe)
        return;
    }

    const float tuning_value = linear_interpolate(g2.tuning_min, g2.tuning_max, norm_in, -1, 1);

#if HAL_LOGGING_ENABLED
    Log_Write_Parameter_Tuning(g.rc_tuning_param, tuning_value, g2.tuning_min, g2.tuning_max);
#endif

    switch(g.rc_tuning_param) {

    // Roll, Pitch tuning
    case TUNING_STABILIZE_ROLL_PITCH_KP:
        attitude_control->get_angle_roll_p().set_kP(tuning_value);
        attitude_control->get_angle_pitch_p().set_kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KP:
        attitude_control->get_rate_roll_pid().set_kP(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KI:
        attitude_control->get_rate_roll_pid().set_kI(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KD:
        attitude_control->get_rate_roll_pid().set_kD(tuning_value);
        attitude_control->get_rate_pitch_pid().set_kD(tuning_value);
        break;

    // Yaw tuning
    case TUNING_STABILIZE_YAW_KP:
        attitude_control->get_angle_yaw_p().set_kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KP:
        attitude_control->get_rate_yaw_pid().set_kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KD:
        attitude_control->get_rate_yaw_pid().set_kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case TUNING_ALTITUDE_HOLD_KP:
        pos_control->get_pos_U_p().set_kP(tuning_value);
        break;

    case TUNING_THROTTLE_RATE_KP:
        pos_control->get_vel_U_pid().set_kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KP:
        pos_control->get_accel_U_pid().set_kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KI:
        pos_control->get_accel_U_pid().set_kI(tuning_value);
        break;

    case TUNING_ACCEL_Z_KD:
        pos_control->get_accel_U_pid().set_kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case TUNING_LOITER_POSITION_KP:
        pos_control->get_pos_NE_p().set_kP(tuning_value);
        break;

    case TUNING_VEL_XY_KP:
        pos_control->get_vel_NE_pid().set_kP(tuning_value);
        break;

    case TUNING_VEL_XY_KI:
        pos_control->get_vel_NE_pid().set_kI(tuning_value);
        break;

    case TUNING_WP_SPEED:
        wp_nav->set_speed_NE_cms(tuning_value);
        break;

#if MODE_ACRO_ENABLED || MODE_SPORT_ENABLED
    // Acro roll pitch rates
    case TUNING_ACRO_RP_RATE:
        g2.command_model_acro_rp.set_rate(tuning_value);
        break;
#endif

#if MODE_ACRO_ENABLED || MODE_DRIFT_ENABLED
    // Acro yaw rate
    case TUNING_ACRO_YAW_RATE:
        g2.command_model_acro_y.set_rate(tuning_value);
        break;
#endif

#if FRAME_CONFIG == HELI_FRAME
    case TUNING_HELI_EXTERNAL_GYRO:
        motors->ext_gyro_gain(tuning_value);
        break;

    case TUNING_RATE_PITCH_FF:
        attitude_control->get_rate_pitch_pid().set_ff(tuning_value);
        break;

    case TUNING_RATE_ROLL_FF:
        attitude_control->get_rate_roll_pid().set_ff(tuning_value);
        break;

    case TUNING_RATE_YAW_FF:
        attitude_control->get_rate_yaw_pid().set_ff(tuning_value);
        break;
#endif

    case TUNING_DECLINATION:
        compass.set_declination(radians(tuning_value), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

#if MODE_CIRCLE_ENABLED
    case TUNING_CIRCLE_RATE:
        circle_nav->set_rate_degs(tuning_value);
        break;
#endif

    case TUNING_RC_FEEL_RP:
        attitude_control->set_input_tc(tuning_value);
        break;

    case TUNING_RATE_PITCH_KP:
        attitude_control->get_rate_pitch_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_PITCH_KI:
        attitude_control->get_rate_pitch_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_PITCH_KD:
        attitude_control->get_rate_pitch_pid().set_kD(tuning_value);
        break;

    case TUNING_RATE_ROLL_KP:
        attitude_control->get_rate_roll_pid().set_kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_KI:
        attitude_control->get_rate_roll_pid().set_kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_KD:
        attitude_control->get_rate_roll_pid().set_kD(tuning_value);
        break;

#if FRAME_CONFIG != HELI_FRAME
    case TUNING_RATE_MOT_YAW_HEADROOM:
        motors->set_yaw_headroom(tuning_value);
        break;
#endif

    case TUNING_RATE_YAW_FILT:
        attitude_control->get_rate_yaw_pid().set_filt_E_hz(tuning_value);
        break;

    case TUNING_SYSTEM_ID_MAGNITUDE:
#if MODE_SYSTEMID_ENABLED
        copter.mode_systemid.set_magnitude(tuning_value);
#endif
        break;

    case TUNING_POS_CONTROL_ANGLE_MAX:
        pos_control->set_lean_angle_max_deg(tuning_value);
        break;

    case TUNING_LOITER_MAX_XY_SPEED:
        loiter_nav->set_speed_max_NE_cms(tuning_value);
        break;
    }
}
