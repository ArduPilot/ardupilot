// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * servos.pde - code to move pitch and yaw servos to attain a target heading or pitch
 */

// init_servos - initialises the servos
void Tracker::init_servos()
{
    // setup antenna control PWM channels
    channel_yaw.set_angle(g.yaw_range * 100/2);        // yaw range is +/- (YAW_RANGE parameter/2) converted to centi-degrees
    channel_pitch.set_angle(g.pitch_range * 100/2);    // pitch range is +/- (PITCH_RANGE parameter/2) converted to centi-degrees

    // move servos to mid position
    channel_yaw.output_trim();
    channel_pitch.output_trim();

    // initialise output to servos
    channel_yaw.calc_pwm();
    channel_pitch.calc_pwm();
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_servo(float pitch)
{
    switch ((enum ServoType)g.servo_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_pitch_onoff_servo(pitch);
        break;

    case SERVO_TYPE_CR:
        update_pitch_cr_servo(pitch);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_pitch_position_servo(pitch);
        break;
    }

    // convert servo_out to radio_out and send to servo
    channel_pitch.calc_pwm();
    channel_pitch.output();
}

/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_position_servo(float pitch)
{
    // degrees(ahrs.pitch) is -90 to 90, where 0 is horizontal
    // pitch argument is -90 to 90, where 0 is horizontal
    // servo_out is in 100ths of a degree
    float ahrs_pitch = ahrs.pitch_sensor*0.01f;
    int32_t angle_err = -(ahrs_pitch - pitch) * 100.0f;
    int32_t pitch_limit_cd = g.pitch_range*100/2;
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
    int32_t new_servo_out = channel_pitch.servo_out + g.pidPitch2Srv.get_pid(angle_err);

    // initialise limit flags
    servo_limit.pitch_lower = false;
    servo_limit.pitch_upper = false;

    // rate limit pitch servo
    if (g.pitch_slew_time > 0.02f) {
        uint16_t max_change = 0.02f * (pitch_limit_cd) / g.pitch_slew_time;
        if (max_change < 1) {
            max_change = 1;
        }
        if (new_servo_out <= channel_pitch.servo_out - max_change) {
            new_servo_out = channel_pitch.servo_out - max_change;
            servo_limit.pitch_lower = true;
        }
        if (new_servo_out >= channel_pitch.servo_out + max_change) {
            new_servo_out = channel_pitch.servo_out + max_change;
            servo_limit.pitch_upper = true;
        }
    }
    channel_pitch.servo_out = new_servo_out;

    // position limit pitch servo
    if (channel_pitch.servo_out <= -pitch_limit_cd) {
        channel_pitch.servo_out = -pitch_limit_cd;
        servo_limit.pitch_lower = true;
    }
    if (channel_pitch.servo_out >= pitch_limit_cd) {
        channel_pitch.servo_out = pitch_limit_cd;
        servo_limit.pitch_upper = true;
    }
}


/**
   update the pitch (elevation) servo. The aim is to drive the boards ahrs pitch to the
   requested pitch, so the board (and therefore the antenna) will be pointing at the target
 */
void Tracker::update_pitch_onoff_servo(float pitch)
{
    // degrees(ahrs.pitch) is -90 to 90, where 0 is horizontal
    // pitch argument is -90 to 90, where 0 is horizontal
    // servo_out is in 100ths of a degree
    float ahrs_pitch = degrees(ahrs.pitch);
    float err = ahrs_pitch - pitch;

    float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
    if (fabsf(err) < acceptable_error) {
        channel_pitch.servo_out = 0;
    } else if (err > 0) {
        // positive error means we are pointing too high, so push the
        // servo down
        channel_pitch.servo_out = -9000;
    } else {
        // negative error means we are pointing too low, so push the
        // servo up
        channel_pitch.servo_out = 9000;
    }
}

/**
   update the pitch for continuous rotation servo
*/
void Tracker::update_pitch_cr_servo(float pitch)
{
    float ahrs_pitch = degrees(ahrs.pitch);
    float err_cd = (pitch - ahrs_pitch) * 100.0f;
    channel_pitch.servo_out = g.pidPitch2Srv.get_pid(err_cd);
}

/**
   update the yaw (azimuth) servo.
 */
void Tracker::update_yaw_servo(float yaw)
{
    switch ((enum ServoType)g.servo_type.get()) {
    case SERVO_TYPE_ONOFF:
        update_yaw_onoff_servo(yaw);
        break;

    case SERVO_TYPE_CR:
        update_yaw_cr_servo(yaw);
        break;

    case SERVO_TYPE_POSITION:
    default:
        update_yaw_position_servo(yaw);
        break;
    }

    // convert servo_out to radio_out and send to servo
    channel_yaw.calc_pwm();
    channel_yaw.output();
}

/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_position_servo(float yaw)
{
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t yaw_cd   = wrap_180_cd(yaw*100);
    int32_t yaw_limit_cd = g.yaw_range*100/2;
    const int16_t margin = 500; // 5 degrees slop

    // Antenna as Ballerina. Use with antenna that do not have continuously rotating servos, ie at some point in rotation
    // the servo limits are reached and the servo has to slew 360 degrees to the 'other side' to keep tracking.
    //
    // This algorithm accounts for the fact that the antenna mount may not be aligned with North
    // (in fact, any alignment is permissable), and that the alignment may change (possibly rapidly) over time
    // (as when the antenna is mounted on a moving, turning vehicle)
    // When the servo is being forced beyond its limits, it rapidly slews to the 'other side' then normal tracking takes over.
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
    int32_t angle_err = wrap_180_cd(ahrs_yaw_cd - yaw_cd);

    /*
      a positive error means that we need to rotate counter-clockwise
      a negative error means that we need to rotate clockwise

      Use our current yawspeed to determine if we are moving in the
      right direction
     */
    int8_t new_slew_dir = slew_dir;

    // get earth frame z-axis rotation rate in radians
    Vector3f earth_rotation = ahrs.get_gyro() * ahrs.get_rotation_body_to_ned();


    bool making_progress;
    if (slew_dir != 0) {
        making_progress = (-slew_dir * earth_rotation.z >= 0);
    } else {
        making_progress = (angle_err * earth_rotation.z >= 0);
    }

    // handle hitting servo limits
    if (abs(channel_yaw.servo_out) == 18000 &&
        labs(angle_err) > margin &&
        making_progress &&
        AP_HAL::millis() - slew_start_ms > g.min_reverse_time*1000) {
        // we are at the limit of the servo and are not moving in the
        // right direction, so slew the other way
        new_slew_dir = -channel_yaw.servo_out / 18000;
        slew_start_ms = AP_HAL::millis();
    }

    /*
      stop slewing and revert to normal control when normal control
      should move us in the right direction
     */
    if (slew_dir * angle_err < -margin) {
        new_slew_dir = 0;
    }

    if (new_slew_dir != slew_dir) {
        tracker.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Slew: %d/%d err=%ld servo=%ld ez=%.3f",
                                  (int)slew_dir, (int)new_slew_dir,
                                  (long)angle_err,
                                  (long)channel_yaw.servo_out,
                                  (double)degrees(ahrs.get_gyro().z));
    }

    slew_dir = new_slew_dir;

    // initialise limit flags
    servo_limit.yaw_lower = false;
    servo_limit.yaw_upper = false;

    int16_t new_servo_out;
    if (slew_dir != 0) {
        new_servo_out = slew_dir * 18000;
        g.pidYaw2Srv.reset_I();
    } else {
        float servo_change = g.pidYaw2Srv.get_pid(angle_err);
        servo_change = constrain_float(servo_change, -18000, 18000);
        new_servo_out = constrain_float(channel_yaw.servo_out - servo_change, -18000, 18000);
    }

    // rate limit yaw servo
    if (g.yaw_slew_time > 0.02f) {
        uint16_t max_change = 0.02f * yaw_limit_cd / g.yaw_slew_time;
        if (max_change < 1) {
            max_change = 1;
        }
        if (new_servo_out <= channel_yaw.servo_out - max_change) {
            new_servo_out = channel_yaw.servo_out - max_change;
            servo_limit.yaw_lower = true;
        }
        if (new_servo_out >= channel_yaw.servo_out + max_change) {
            new_servo_out = channel_yaw.servo_out + max_change;
            servo_limit.yaw_upper = true;
        }
    }
    channel_yaw.servo_out = new_servo_out;

    // position limit pitch servo
    if (channel_yaw.servo_out <= -yaw_limit_cd) {
        channel_yaw.servo_out = -yaw_limit_cd;
        servo_limit.yaw_lower = true;
    }
    if (channel_yaw.servo_out >= yaw_limit_cd) {
        channel_yaw.servo_out = yaw_limit_cd;
        servo_limit.yaw_upper = true;
    }
}


/**
   update the yaw (azimuth) servo. The aim is to drive the boards ahrs
   yaw to the requested yaw, so the board (and therefore the antenna)
   will be pointing at the target
 */
void Tracker::update_yaw_onoff_servo(float yaw)
{
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t yaw_cd   = wrap_180_cd(yaw*100);
    int32_t err_cd = wrap_180_cd(ahrs_yaw_cd - yaw_cd);
    float err = err_cd * 0.01f;

    float acceptable_error = g.onoff_yaw_rate * g.onoff_yaw_mintime;
    if (fabsf(err) < acceptable_error) {
        channel_yaw.servo_out = 0;
    } else if (err > 0) {
        // positive error means we are clockwise of the target, so
        // move anti-clockwise
        channel_yaw.servo_out = -18000;
    } else {
        // negative error means we are anti-clockwise of the target, so
        // move clockwise
        channel_yaw.servo_out = 18000;
    }
}

/**
   update the yaw continuous rotation servo
 */
void Tracker::update_yaw_cr_servo(float yaw)
{
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    float yaw_cd = wrap_180_cd_float(yaw*100.0f);
    float err_cd = wrap_180_cd(yaw_cd - (float)ahrs_yaw_cd);

    channel_yaw.servo_out = g.pidYaw2Srv.get_pid(err_cd);
}
