#include "Copter.h"

/*
 * Function to update various parameters in flight using the ch6 tuning knob
 * This should not be confused with the AutoTune feature which can bve found in control_autotune.cpp
 */

// tuning - updates parameters based on the ch6 tuning knob's position
//  should be called at 3.3hz
void Copter::tuning() {
    RC_Channel *rc6 = RC_Channels::rc_channel(CH_6);

    // exit immediately if not using tuning function, or when radio failsafe is invoked, so tuning values are not set to zero    
    if ((g.radio_tuning <= 0) || failsafe.radio || failsafe.radio_counter != 0 || rc6->get_radio_in() == 0) {
        return;
    }

    uint16_t radio_in = rc6->get_radio_in();
    float v = constrain_float((radio_in - rc6->get_radio_min()) / float(rc6->get_radio_max() - rc6->get_radio_min()), 0, 1);
    int16_t control_in = g.radio_tuning_low + v * (g.radio_tuning_high - g.radio_tuning_low);
    float tuning_value = control_in / 1000.0f;
    
    // Tuning Value should never be outside the bounds of the specified low and high value
    tuning_value = constrain_float(tuning_value, g.radio_tuning_low/1000.0f, g.radio_tuning_high/1000.0f);

    Log_Write_Parameter_Tuning(g.radio_tuning, tuning_value, control_in, g.radio_tuning_low, g.radio_tuning_high);

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
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav->set_speed_xy(control_in);
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
        motors->ext_gyro_gain((float)control_in / 1000.0f);
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
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case TUNING_CIRCLE_RATE:
        // set circle rate up to approximately 45 deg/sec in either direction
        circle_nav->set_rate((float)control_in/25.0f-20.0f);
        break;

    case TUNING_RANGEFINDER_GAIN:
        // set rangefinder gain
        g.rangefinder_gain.set(tuning_value);
        break;

#if 0
        // disabled for now - we need accessor functions
    case TUNING_EKF_VERTICAL_POS:
        // Tune the EKF that is being used
        // EKF's baro vs accel (higher rely on accels more, baro impact is reduced)
        if (!ahrs.get_NavEKF2().enabled()) {
            ahrs.get_NavEKF()._gpsVertPosNoise = tuning_value;
        } else {
            ahrs.get_NavEKF2()._gpsVertPosNoise = tuning_value;
        }
        break;

    case TUNING_EKF_HORIZONTAL_POS:
        // EKF's gps vs accel (higher rely on accels more, gps impact is reduced)
        if (!ahrs.get_NavEKF2().enabled()) {
            ahrs.get_NavEKF()._gpsHorizPosNoise = tuning_value;
        } else {
            ahrs.get_NavEKF2()._gpsHorizPosNoise = tuning_value;
        }
        break;

    case TUNING_EKF_ACCEL_NOISE:
        // EKF's accel noise (lower means trust accels more, gps & baro less)
        if (!ahrs.get_NavEKF2().enabled()) {
            ahrs.get_NavEKF()._accNoise = tuning_value;
        } else {
            ahrs.get_NavEKF2()._accNoise = tuning_value;
        }
        break;
#endif

    case TUNING_RC_FEEL_RP:
        // convert from control_in to input time constant
        attitude_control->set_input_tc(1.0f / (2.f + MAX((control_in/100.0f),0.0f)));
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
        motors->set_yaw_headroom(tuning_value*1000);
        break;
#endif

     case TUNING_RATE_YAW_FILT:
         attitude_control->get_rate_yaw_pid().filt_hz(tuning_value);
         break;

     case TUNING_WINCH: {
         float desired_rate = 0.0f;
         if (v > 0.6f) {
             desired_rate = g2.winch.get_rate_max() * (v - 0.6f) / 0.4f;
         }
         if (v < 0.4f) {
             desired_rate = g2.winch.get_rate_max() * (v - 0.4) / 0.4f;
         }
         g2.winch.set_desired_rate(desired_rate);
         break;
         }
     }
}
