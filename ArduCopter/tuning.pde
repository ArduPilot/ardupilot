/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * tuning.pde - function to update various parameters in flight using the ch6 tuning knob
 *      This should not be confused with the AutoTune feature which can bve found in control_autotune.pde
 */

// tuning - updates parameters based on the ch6 tuning knob's position
//  should be called at 3.3hz
static void tuning() {

    // exit immediately if not tuning of when radio failsafe is invoked so tuning values are not set to zero
    if ((g.radio_tuning <= 0) || failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    float tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case CH6_STABILIZE_ROLL_PITCH_KP:
        g.p_stabilize_roll.kP(tuning_value);
        g.p_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    // Yaw tuning
    case CH6_STABILIZE_YAW_KP:
        g.p_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case CH6_ALTITUDE_HOLD_KP:
        g.p_alt_hold.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KP:
        g.p_vel_z.kP(tuning_value);
        break;

    case CH6_ACCEL_Z_KP:
        g.pid_accel_z.kP(tuning_value);
        break;

    case CH6_ACCEL_Z_KI:
        g.pid_accel_z.kI(tuning_value);
        break;

    case CH6_ACCEL_Z_KD:
        g.pid_accel_z.kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case CH6_LOITER_POSITION_KP:
        g.p_pos_xy.kP(tuning_value);
        break;

    case CH6_VEL_XY_KP:
        g.pi_vel_xy.kP(tuning_value);
        break;

    case CH6_VEL_XY_KI:
        g.pi_vel_xy.kI(tuning_value);
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_speed_xy(g.rc_6.control_in);
        break;

    // Acro roll pitch gain
    case CH6_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case CH6_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain(g.rc_6.control_in);
        break;

    case CH6_RATE_PITCH_FF:
        g.pid_rate_pitch.ff(tuning_value);
        break;

    case CH6_RATE_ROLL_FF:
        g.pid_rate_roll.ff(tuning_value);
        break;

    case CH6_RATE_YAW_FF:
        g.pid_rate_yaw.ff(tuning_value);
        break;
#endif

    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        circle_nav.set_rate(g.rc_6.control_in/25-20);   // allow approximately 45 degree turn rate in either direction
        break;

    case CH6_SONAR_GAIN:
        // set sonar gain
        g.sonar_gain.set(tuning_value);
        break;

#if 0
        // disabled for now - we need accessor functions
    case CH6_EKF_VERTICAL_POS:
        // EKF's baro vs accel (higher rely on accels more, baro impact is reduced)
        ahrs.get_NavEKF()._gpsVertPosNoise = tuning_value;
        break;

    case CH6_EKF_HORIZONTAL_POS:
        // EKF's gps vs accel (higher rely on accels more, gps impact is reduced)
        ahrs.get_NavEKF()._gpsHorizPosNoise = tuning_value;
        break;

    case CH6_EKF_ACCEL_NOISE:
        // EKF's accel noise (lower means trust accels more, gps & baro less)
        ahrs.get_NavEKF()._accNoise = tuning_value;
        break;
#endif

    case CH6_RC_FEEL_RP:
        // roll-pitch input smoothing
        g.rc_feel_rp = g.rc_6.control_in / 10;
        break;

    case CH6_RATE_PITCH_KP:
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_PITCH_KI:
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_PITCH_KD:
        g.pid_rate_pitch.kD(tuning_value);
        break;

    case CH6_RATE_ROLL_KP:
        g.pid_rate_roll.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_KI:
        g.pid_rate_roll.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_KD:
        g.pid_rate_roll.kD(tuning_value);
        break;

    case CH6_RATE_MOT_YAW_HEADROOM:
        motors.set_yaw_headroom(tuning_value*1000);
        break;

     case CH6_RATE_YAW_FILT:
         g.pid_rate_yaw.filt_hz(tuning_value);
         break;
    }
}
