#include "Copter.h"

/*
 * Function to update various parameters in flight using the ch6 tuning knob
 * This should not be confused with the AutoTune feature which can bve found in control_autotune.cpp
 */

// tuning - updates parameters based on the ch6 tuning knob's position
//  should be called at 3.3hz
void Copter::tuning()
{
    const RC_Channel *rc6 = rc().channel(CH_6);

    // exit immediately if the tuning function is not set or min and max are both zero
    if ((g.radio_tuning <= 0) || (is_zero(g2.tuning_min.get()) && is_zero(g2.tuning_max.get()))) {
        return;
    }

    // exit immediately when radio failsafe is invoked or transmitter has not been turned on
    if (failsafe.radio || failsafe.radio_counter != 0 || rc6->get_radio_in() == 0) {
        return;
    }

    // exit immediately if a function is assigned to channel 6
    if ((RC_Channel::aux_func_t)rc6->option.get() != RC_Channel::AUX_FUNC::DO_NOTHING) {
        return;
    }

    const uint16_t radio_in = rc6->get_radio_in();
    float tuning_value = linear_interpolate(g2.tuning_min, g2.tuning_max, radio_in, rc6->get_radio_min(), rc6->get_radio_max());
    Log_Write_Parameter_Tuning(g.radio_tuning, tuning_value, g2.tuning_min, g2.tuning_max);

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case TUNING_STABILIZE_ROLL_PITCH_KP:
        attitude_control->get_angle_roll_p().kP(tuning_value);
        attitude_control->get_angle_pitch_p().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KP:
        attitude_control->get_rate_roll_pid().kP(tuning_value);
        attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KI:
        attitude_control->get_rate_roll_pid().kI(tuning_value);
        attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KD:
        attitude_control->get_rate_roll_pid().kD(tuning_value);
        attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    // Yaw tuning
    case TUNING_STABILIZE_YAW_KP:
        attitude_control->get_angle_yaw_p().kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KP:
        attitude_control->get_rate_yaw_pid().kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KD:
        attitude_control->get_rate_yaw_pid().kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case TUNING_ALTITUDE_HOLD_KP:
        pos_control->get_pos_z_p().kP(tuning_value);
        break;

    case TUNING_THROTTLE_RATE_KP:
        pos_control->get_vel_z_p().kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KP:
        pos_control->get_accel_z_pid().kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KI:
        pos_control->get_accel_z_pid().kI(tuning_value);
        break;

    case TUNING_ACCEL_Z_KD:
        pos_control->get_accel_z_pid().kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case TUNING_LOITER_POSITION_KP:
        pos_control->get_pos_xy_p().kP(tuning_value);
        break;

    case TUNING_VEL_XY_KP:
        pos_control->get_vel_xy_pid().kP(tuning_value);
        break;

    case TUNING_VEL_XY_KI:
        pos_control->get_vel_xy_pid().kI(tuning_value);
        break;

    case TUNING_WP_SPEED:
        wp_nav->set_speed_xy(tuning_value);
        break;

    // Acro roll pitch gain
    case TUNING_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case TUNING_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

#if FRAME_CONFIG == HELI_FRAME
    case TUNING_HELI_EXTERNAL_GYRO:
        motors->ext_gyro_gain(tuning_value);
        break;

    case TUNING_RATE_PITCH_FF:
        attitude_control->get_rate_pitch_pid().ff(tuning_value);
        break;

    case TUNING_RATE_ROLL_FF:
        attitude_control->get_rate_roll_pid().ff(tuning_value);
        break;

    case TUNING_RATE_YAW_FF:
        attitude_control->get_rate_yaw_pid().ff(tuning_value);
        break;
#endif

    case TUNING_DECLINATION:
        compass.set_declination(ToRad(tuning_value), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

#if MODE_CIRCLE_ENABLED == ENABLED
    case TUNING_CIRCLE_RATE:
        circle_nav->set_rate(tuning_value);
        break;
#endif

#if RANGEFINDER_ENABLED == ENABLED
    case TUNING_RANGEFINDER_GAIN:
        // set rangefinder gain
        g.rangefinder_gain.set(tuning_value);
        break;
#endif

    case TUNING_RC_FEEL_RP:
        attitude_control->set_input_tc(tuning_value);
        break;

    case TUNING_RATE_PITCH_KP:
        attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_RATE_PITCH_KI:
        attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_RATE_PITCH_KD:
        attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    case TUNING_RATE_ROLL_KP:
        attitude_control->get_rate_roll_pid().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_KI:
        attitude_control->get_rate_roll_pid().kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_KD:
        attitude_control->get_rate_roll_pid().kD(tuning_value);
        break;

#if FRAME_CONFIG != HELI_FRAME
    case TUNING_RATE_MOT_YAW_HEADROOM:
        motors->set_yaw_headroom(tuning_value);
        break;
#endif

     case TUNING_RATE_YAW_FILT:
         attitude_control->get_rate_yaw_pid().filt_E_hz(tuning_value);
         break;

     case TUNING_SYSTEM_ID_MAGNITUDE:
#if MODE_SYSTEMID_ENABLED == ENABLED
         copter.mode_systemid.set_magnitude(tuning_value);
#endif
         break;
    }
}
