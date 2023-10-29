#include "mode.h"
#include "Plane.h"

bool ModeAcro::_enter()
{
    acro_state.locked_roll = false;
    acro_state.locked_pitch = false;
    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));
    return true;
}

void ModeAcro::update()
{
    // handle locked/unlocked control
    if (acro_state.locked_roll) {
        plane.nav_roll_cd = acro_state.locked_roll_err;
    } else {
        plane.nav_roll_cd = ahrs.roll_sensor;
    }
    if (acro_state.locked_pitch) {
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
    } else {
        plane.nav_pitch_cd = ahrs.pitch_sensor;
    }
}

void ModeAcro::run()
{
    if (plane.g.acro_locking == 2 && plane.g.acro_yaw_rate > 0 &&
        plane.yawController.rate_control_enabled()) {
        // we can do 3D acro locking
        stabilize_quaternion();
        return;
    }

    // Normal acro
    stabilize();
}

/*
  this is the ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
void ModeAcro::stabilize()
{
    const float speed_scaler = plane.get_speed_scaler();
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;

    IGNORE_RETURN(ahrs.get_quaternion(acro_state.q));

    /*
      check for special roll handling near the pitch poles
     */
    if (plane.g.acro_locking && is_zero(roll_rate)) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * plane.G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        plane.nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilize' to true, which disables the roll integrator
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_servo_out(roll_error_cd,
                                                                                             speed_scaler,
                                                                                             true, false));
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(roll_rate,  speed_scaler));
    }

    if (plane.g.acro_locking && is_zero(pitch_rate)) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        plane.nav_pitch_cd = acro_state.locked_pitch_cd;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_servo_out(plane.nav_pitch_cd - ahrs.pitch_sensor,
                                                                                               speed_scaler,
                                                                                               false, false));
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(pitch_rate, speed_scaler));
    }

    float rudder_output;
    if (plane.g.acro_yaw_rate > 0 && plane.yawController.rate_control_enabled()) {
        // user has asked for yaw rate control with yaw rate scaled by ACRO_YAW_RATE
        const float rudd_expo = plane.rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * plane.g.acro_yaw_rate;
        rudder_output = plane.yawController.get_rate_out(yaw_rate,  speed_scaler, false);
    } else if (plane.flight_option_enabled(FlightOptions::ACRO_YAW_DAMPER)) {
        // use yaw controller
        rudder_output = plane.calc_nav_yaw_coordinated();
    } else {
        /*
          manual rudder
        */
        rudder_output = plane.rudder_input();
    }

    output_rudder_and_steering(rudder_output);

}

/*
  quaternion based acro stabilization with continuous locking. Enabled with ACRO_LOCKING=2
 */
void ModeAcro::stabilize_quaternion()
{
    const float speed_scaler = plane.get_speed_scaler();
    auto &q = acro_state.q;
    const float rexpo = plane.roll_in_expo(true);
    const float pexpo = plane.pitch_in_expo(true);
    const float yexpo = plane.rudder_in_expo(true);

    // get pilot desired rates
    float roll_rate = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
    float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;
    float yaw_rate = (yexpo/SERVO_MAX) * plane.g.acro_yaw_rate;
    bool roll_active = !is_zero(roll_rate);
    bool pitch_active = !is_zero(pitch_rate);
    bool yaw_active = !is_zero(yaw_rate);

    // integrate target attitude
    Vector3f r{ float(radians(roll_rate)), float(radians(pitch_rate)), float(radians(yaw_rate)) };
    r *= plane.G_Dt;
    q.rotate_fast(r);
    q.normalize();

    // fill in target roll/pitch for GCS/logs
    plane.nav_roll_cd = degrees(q.get_euler_roll())*100;
    plane.nav_pitch_cd = degrees(q.get_euler_pitch())*100;

    // get AHRS attitude
    Quaternion ahrs_q;
    IGNORE_RETURN(ahrs.get_quaternion(ahrs_q));

    // zero target if not flying, no stick input and zero throttle
    if (is_zero(plane.get_throttle_input()) &&
        !plane.is_flying() &&
        is_zero(roll_rate) &&
        is_zero(pitch_rate) &&
        is_zero(yaw_rate)) {
        // cope with sitting on the ground with neutral sticks, no throttle
        q = ahrs_q;
    }

    // get error in attitude
    Quaternion error_quat = ahrs_q.inverse() * q;
    Vector3f error_angle1;
    error_quat.to_axis_angle(error_angle1);

    // don't let too much error build up, limit to 0.2s
    const float max_error_t = 0.2;
    float max_err_roll_rad  = radians(plane.g.acro_roll_rate*max_error_t);
    float max_err_pitch_rad = radians(plane.g.acro_pitch_rate*max_error_t);
    float max_err_yaw_rad   = radians(plane.g.acro_yaw_rate*max_error_t);

    if (!roll_active && acro_state.roll_active_last) {
        max_err_roll_rad = 0;
    }
    if (!pitch_active && acro_state.pitch_active_last) {
        max_err_pitch_rad = 0;
    }
    if (!yaw_active && acro_state.yaw_active_last) {
        max_err_yaw_rad = 0;
    }

    Vector3f desired_rates = error_angle1;
    desired_rates.x = constrain_float(desired_rates.x, -max_err_roll_rad, max_err_roll_rad);
    desired_rates.y = constrain_float(desired_rates.y, -max_err_pitch_rad, max_err_pitch_rad);
    desired_rates.z = constrain_float(desired_rates.z, -max_err_yaw_rad, max_err_yaw_rad);

    // correct target based on max error
    q.rotate_fast(desired_rates - error_angle1);
    q.normalize();

    // convert to desired body rates
    desired_rates.x /= plane.rollController.tau();
    desired_rates.y /= plane.pitchController.tau();
    desired_rates.z /= plane.pitchController.tau(); // no yaw tau parameter, use pitch

    desired_rates *= degrees(1.0);

    if (roll_active) {
        desired_rates.x = roll_rate;
    }
    if (pitch_active) {
        desired_rates.y = pitch_rate;
    }
    if (yaw_active) {
        desired_rates.z = yaw_rate;
    }

    // call to rate controllers
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  plane.rollController.get_rate_out(desired_rates.x, speed_scaler));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitchController.get_rate_out(desired_rates.y, speed_scaler));
    output_rudder_and_steering(plane.yawController.get_rate_out(desired_rates.z,  speed_scaler, false));

    acro_state.roll_active_last = roll_active;
    acro_state.pitch_active_last = pitch_active;
    acro_state.yaw_active_last = yaw_active;
}
