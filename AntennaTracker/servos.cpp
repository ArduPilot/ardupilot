#include "Tracker.h"

/*
 * Code to move pitch and yaw servos to attain a target heading or pitch
 */

// init_servos - initialises the servos
void Tracker::init_servos()
{
    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    SRV_Channels::set_default_function(CH_YAW, SRV_Channel::k_tracker_yaw);
    SRV_Channels::set_default_function(CH_PITCH, SRV_Channel::k_tracker_pitch);

    // yaw range is +/- (YAW_RANGE parameter/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_yaw, g.yaw_range * 100/2);        

    // pitch range is +/- (PITCH_MIN/MAX parameters/2) converted to centi-degrees
    SRV_Channels::set_angle(SRV_Channel::k_tracker_pitch, (-g.pitch_min+g.pitch_max) * 100/2);

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();

    yaw_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
    pitch_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_servo(float pitch)
{
    switch ((enum ServoType)g.servo_pitch_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_pitch_onoff_servo(pitch);
        break;

    case SERVO_TYPE_CR:
        update_pitch_cr_servo(pitch);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_pitch_position_servo();
        break;
    }

    // convert servo_out to radio_out and send to servo
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_position_servo()
{
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;
    // Need to configure your servo so that increasing servo_out causes increase in pitch/elevation (ie pointing higher into the sky,
    // above the horizon. On my antenna tracker this requires the pitch/elevation servo to be reversed
    // param set RC2_REV -1
    //
    // The pitch servo (RC channel 2) is configured for servo_out of -9000-0-9000 servo_out,
    // which will drive the servo from RC2_MIN to RC2_MAX usec pulse width.
    // Therefore, you must set RC2_MIN and RC2_MAX so that your servo drives the antenna altitude between -90 to 90 exactly
    // To drive my HS-645MG servos through their full 180 degrees of rotational range, I have to set:
    // param set RC2_MAX 2540
    // param set RC2_MIN 640
    //
    // You will also need to tune the pitch PID to suit your antenna and servos. I use:
    // PITCH2SRV_P      0.100000
    // PITCH2SRV_I      0.020000
    // PITCH2SRV_D      0.000000
    // PITCH2SRV_IMAX   4000.000000

    // calculate new servo position
    g.pidPitch2Srv.set_input_filter_all(nav_status.angle_error_pitch);
    int32_t new_servo_out = SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_pitch) + g.pidPitch2Srv.get_pid();

    // position limit pitch servo
    if (new_servo_out <= pitch_min_cd) {
        new_servo_out = pitch_min_cd;
        g.pidPitch2Srv.reset_I();
    }
    if (new_servo_out >= pitch_max_cd) {
        new_servo_out = pitch_max_cd;
        g.pidPitch2Srv.reset_I();
    }
    // rate limit pitch servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, new_servo_out);

    if (pitch_servo_out_filt_init) {
        pitch_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        pitch_servo_out_filt.reset(new_servo_out);
        pitch_servo_out_filt_init = true;
    }
}


/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_onoff_servo(float pitch)
{
    int32_t pitch_min_cd = g.pitch_min*100;
    int32_t pitch_max_cd = g.pitch_max*100;

    float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
    if (fabsf(nav_status.angle_error_pitch) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
    } else if ((nav_status.angle_error_pitch > 0) && (pitch*100>pitch_min_cd)) {
        // positive error means we are pointing too low, so push the
        // servo up
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, -9000);
    } else if (pitch*100<pitch_max_cd) {
        // negative error means we are pointing too high, so push the
        // servo down
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 9000);
    }
}

/**
   update the pitch for continuous rotation servo
*/
void Tracker::update_pitch_cr_servo(float pitch)
{
    g.pidPitch2Srv.set_input_filter_all(nav_status.angle_error_pitch);
    const float pitch_out = constrain_float(g.pidPitch2Srv.get_pid(), -(-g.pitch_min+g.pitch_max) * 100/2, (-g.pitch_min+g.pitch_max) * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, pitch_out);
}

/**
   update the yaw (azimuth) servo.
 */
void Tracker::update_yaw_servo(float yaw)
{
	switch ((enum ServoType)g.servo_yaw_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_yaw_onoff_servo(yaw);
        break;

    case SERVO_TYPE_CR:
        update_yaw_cr_servo(yaw);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_yaw_position_servo();
        break;
    }

    // convert servo_out to radio_out and send to servo
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
}

/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_position_servo()
{
    int32_t yaw_limit_cd = g.yaw_range*100/2;

    // Antenna as Ballerina. Use with antenna that do not have continuously rotating servos, ie at some point in rotation
    // the servo limits are reached and the servo has to slew 360 degrees to the 'other side' to keep tracking.
    //
    // This algorithm accounts for the fact that the antenna mount may not be aligned with North
    // (in fact, any alignment is permissible), and that the alignment may change (possibly rapidly) over time
    // (as when the antenna is mounted on a moving, turning vehicle)
    //
    // With my antenna mount, large pwm output drives the antenna anticlockise, so need:
    // param set RC1_REV -1
    // to reverse the servo. Yours may be different
    //
    // You MUST set RC1_MIN and RC1_MAX so that your servo drives the antenna azimuth from -180 to 180 relative
    // to the mount.
    // To drive my HS-645MG servos through their full 180 degrees of rotational range and therefore the
    // antenna through a full 360 degrees, I have to set:
    // param set RC1_MAX 2380
    // param set RC1_MIN 680
    // According to the specs at https://www.servocity.com/html/hs-645mg_ultra_torque.html,
    // that should be 600 through 2400, but the azimuth gearing in my antenna pointer is not exactly 2:1

    /*
      a positive error means that we need to rotate clockwise
      a negative error means that we need to rotate counter-clockwise

      Use our current yawspeed to determine if we are moving in the
      right direction
     */

    g.pidYaw2Srv.set_input_filter_all(nav_status.angle_error_yaw);
    float servo_change = g.pidYaw2Srv.get_pid();
    servo_change = constrain_float(servo_change, -18000, 18000);
    float new_servo_out = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_yaw) + servo_change, -18000, 18000);

    // position limit yaw servo
    if (new_servo_out <= -yaw_limit_cd) {
        new_servo_out = -yaw_limit_cd;
        g.pidYaw2Srv.reset_I();
    }
    if (new_servo_out >= yaw_limit_cd) {
        new_servo_out = yaw_limit_cd;
        g.pidYaw2Srv.reset_I();
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, new_servo_out);

    if (yaw_servo_out_filt_init) {
        yaw_servo_out_filt.apply(new_servo_out, G_Dt);
    } else {
        yaw_servo_out_filt.reset(new_servo_out);
        yaw_servo_out_filt_init = true;
    }
}


/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_onoff_servo(float yaw)
{
    float acceptable_error = g.onoff_yaw_rate * g.onoff_yaw_mintime;
    if (fabsf(nav_status.angle_error_yaw * 0.01f) < acceptable_error) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
    } else if (nav_status.angle_error_yaw * 0.01f > 0) {
        // positive error means we are counter-clockwise of the target, so
        // move clockwise
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 18000);
    } else {
        // negative error means we are clockwise of the target, so
        // move counter-clockwise
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, -18000);
    }
}

/**
   update the yaw continuous rotation servo
 */
void Tracker::update_yaw_cr_servo(float yaw)
{
    g.pidYaw2Srv.set_input_filter_all(nav_status.angle_error_yaw);
    const float yaw_out = constrain_float(-g.pidYaw2Srv.get_pid(), -g.yaw_range * 100/2, g.yaw_range * 100/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, yaw_out);
}
