/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  control code for tailsitters. Enabled by setting Q_FRAME_CLASS=10 
  or by setting Q_TAILSIT_MOTMX nonzero and Q_FRAME_CLASS and Q_FRAME_TYPE
  to a configuration supported by AP_MotorsMatrix
 */

#include <math.h>
#include "Plane.h"

/*
  return true when flying a tailsitter
 */
bool QuadPlane::is_tailsitter(void) const
{
    return available()
        && ((frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) || (tailsitter.motor_mask != 0))
        && (tilt.tilt_type != TILT_TYPE_BICOPTER);
}

/*
  return true when flying a control surface only tailsitter
 */
bool QuadPlane::is_control_surface_tailsitter(void) const
{
    return frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER
           && ( is_zero(tailsitter.vectored_hover_gain) || !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));
}

/*
  check if we are flying as a tailsitter
 */
bool QuadPlane::tailsitter_active(void)
{
    if (!is_tailsitter()) {
        return false;
    }
    if (in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (transition_state == TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/*
  run output for tailsitters
 */
void QuadPlane::tailsitter_output(void)
{
    if (!is_tailsitter() || motor_test.running) {
        // if motor test is running we don't want to overwrite it with output_motor_mask or motors_output
        return;
    }

    float tilt_left = 0.0f;
    float tilt_right = 0.0f;



    // handle forward flight modes and transition to VTOL modes
    if (!tailsitter_active() || in_tailsitter_vtol_transition()) {
        // get FW controller throttle demand and mask of motors enabled during forward flight
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        if (hal.util->get_soft_armed() && in_tailsitter_vtol_transition() && !throttle_wait && is_flying()) {
            /*
              during transitions to vtol mode set the throttle to
              hover thrust, center the rudder and set the altitude controller
              integrator to the same throttle level
              convert the hover throttle to the same output that would result if used via AP_Motors
              apply expo, battery scaling and SPIN min/max. 
            */
            throttle = motors->thrust_to_actuator(motors->get_throttle_hover()) * 100;
            throttle = MAX(throttle,plane.aparm.throttle_cruise.get());

            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);

            // override AP_MotorsTailsitter throttles during back transition

            // apply PWM min and MAX to throttle left and right, just as via AP_Motors
            uint16_t throttle_pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * throttle * 0.01f;
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, throttle_pwm);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, throttle_pwm);

            // throttle output is not used by AP_Motors so might have diffrent PWM range, set scaled
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
        }

        if (!assisted_flight) {
            // set AP_MotorsMatrix throttles for forward flight
            motors->output_motor_mask(throttle * 0.01f, tailsitter.motor_mask, plane.rudder_dt);

            // in forward flight: set motor tilt servos and throttles using FW controller
            if (tailsitter.vectored_forward_gain > 0) {
                // remove scaling from surface speed scaling and apply throttle scaling
                const float scaler = plane.control_mode == &plane.mode_manual?1:(tilt_throttle_scaling() / plane.get_speed_scaler());
                // thrust vectoring in fixed wing flight
                float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
                float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
                tilt_left  = (elevator + aileron) * tailsitter.vectored_forward_gain * scaler;
                tilt_right = (elevator - aileron) * tailsitter.vectored_forward_gain * scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
            return;
        }
    }

    // handle Copter controller
    // the MultiCopter rate controller has already been run in an earlier call
    // to motors_output() from quadplane.update(), unless we are in assisted flight
    // tailsitter in TRANSITION_ANGLE_WAIT_FW is not really in assisted flight, its still in a VTOL mode
    if (assisted_flight && (transition_state != TRANSITION_ANGLE_WAIT_FW)) {
        hold_stabilize(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01f);
        motors_output(true);

        if ((options & OPTION_TAILSIT_Q_ASSIST_MOTORS_ONLY) != 0) {
            // only use motors for Q assist, control surfaces remain under plane control
            // zero copter I terms and use plane
            attitude_control->reset_rate_controller_I_terms();

            // output tilt motors
            tilt_left = 0.0f;
            tilt_right = 0.0f;
            if (tailsitter.vectored_hover_gain > 0) {
                const float hover_throttle = motors->get_throttle_hover();
                const float throttle = motors->get_throttle();
                float throttle_scaler = tailsitter.throttle_scale_max;
                if (is_positive(throttle)) {
                    throttle_scaler = constrain_float(hover_throttle / throttle, tailsitter.gain_scaling_min, tailsitter.throttle_scale_max);
                }
                tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * tailsitter.vectored_hover_gain * throttle_scaler;
                tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * tailsitter.vectored_hover_gain * throttle_scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);


            // skip remainder of the function that overwrites plane control surface outputs with copter
            return;
        }
    } else {
        motors_output(false);
    }

    // In full Q assist it is better to use cotper I and zero plane
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    // pull in copter control outputs
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (motors->get_yaw()+motors->get_yaw_ff())*-SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (motors->get_pitch()+motors->get_pitch_ff())*SERVO_MAX);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (motors->get_roll()+motors->get_roll_ff())*SERVO_MAX);

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    tilt_left = 0.0f;
    tilt_right = 0.0f;
    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        float des_pitch_cd = attitude_control->get_att_target_euler_cd().y;
        int32_t pitch_error_cd = (des_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = 0;
        if (!is_zero(extra_pitch) && in_vtol_mode()) {
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * SERVO_MAX;
        }
        tilt_left  = extra_elevator + tilt_left * tailsitter.vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * tailsitter.vectored_hover_gain;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

    // Check for saturated limits
    bool tilt_lim = (labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft)) == SERVO_MAX) || (labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) == SERVO_MAX);
    bool roll_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_rudder)) == SERVO_MAX;
    bool pitch_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_elevator)) == SERVO_MAX;
    bool yaw_lim = labs(SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t::k_aileron)) == SERVO_MAX;

    if (roll_lim) {
        motors->limit.roll = true;
    }
    if (pitch_lim || tilt_lim) {
        motors->limit.pitch = true;
    }
    if (yaw_lim || tilt_lim) {
        motors->limit.yaw = true;
    }

    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter.input_mask_chan-1) > RC_Channel::AUX_PWM_TRIGGER_HIGH) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_fw_complete(void)
{
    if (!hal.util->get_soft_armed()) {
        // instant trainsition when disarmed, no message
        return true;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle_fw*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        return true;
    }
    if (labs(ahrs_view->roll_sensor) > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, roll error");
        return true;
    }
    if (AP_HAL::millis() - transition_start_ms > ((tailsitter.transition_angle_fw+(transition_initial_pitch*0.01f))/tailsitter.transition_rate_fw)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, timeout");
        return true;
    }
    // still waiting
    return false;
}


/*
  return true when we have completed enough of a transition to switch to VTOL control
 */
bool QuadPlane::tailsitter_transition_vtol_complete(void) const
{
    if (!hal.util->get_soft_armed()) {
        // instant trainsition when disarmed, no message
        return true;
    }
    // for vectored tailsitters at zero pilot throttle
    if ((plane.quadplane.get_pilot_throttle() < .05f) && plane.quadplane._is_vectored) {
        // if we are not moving (hence on the ground?) or don't know
        // transition immediately to tilt motors up and prevent prop strikes
        if (ahrs.groundspeed() < 1.0f) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done, zero throttle");
            return true;
        }
    }
    const float trans_angle = get_tailsitter_transition_angle_vtol();
    if (labs(plane.ahrs.pitch_sensor) > trans_angle*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done");
        return true;
    }
    int32_t roll_cd = labs(plane.ahrs.roll_sensor);
    if (plane.fly_inverted()) {
        roll_cd = 18000 - roll_cd;
    }
    if (roll_cd > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, roll error");
        return true;
    }
    if (AP_HAL::millis() - transition_start_ms >  ((trans_angle-(transition_initial_pitch*0.01f))/tailsitter.transition_rate_vtol)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, timeout");
        return true;
    }
    // still waiting
    attitude_control->reset_rate_controller_I_terms();
    return false;
}

// handle different tailsitter input types
void QuadPlane::tailsitter_check_input(void)
{
    if (tailsitter_active() &&
        (tailsitter.input_type & TAILSITTER_INPUT_PLANE)) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/*
  return true if we are a tailsitter transitioning to VTOL flight
 */
bool QuadPlane::in_tailsitter_vtol_transition(uint32_t now) const
{
    if (!is_tailsitter() || !in_vtol_mode()) {
        return false;
    }
    if (transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
        return true;
    }
    if ((now != 0) && ((now - last_vtol_mode_ms) > 1000)) {
        // only just come out of forward flight
        return true;
    }
    return false;
}

/*
  return true if we are a tailsitter in FW flight
 */
bool QuadPlane::is_tailsitter_in_fw_flight(void) const
{
    return is_tailsitter() && !in_vtol_mode() && transition_state == TRANSITION_DONE;
}

/*
 return the tailsitter.transition_angle_vtol value if non zero, otherwise returns the tailsitter.transition_angle_fw value.
 */
int8_t QuadPlane::get_tailsitter_transition_angle_vtol() const
{
    if (tailsitter.transition_angle_vtol == 0) {
        return tailsitter.transition_angle_fw;
    }
    return tailsitter.transition_angle_vtol;
}


/*
  account for speed scaling of control surfaces in VTOL modes
*/
void QuadPlane::tailsitter_speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float spd_scaler = 1.0f;

    // Scaleing with throttle
    float throttle_scaler = tailsitter.throttle_scale_max;
    if (is_positive(throttle)) {
        throttle_scaler = constrain_float(hover_throttle / throttle, tailsitter.gain_scaling_min, tailsitter.throttle_scale_max);
    }

    if ((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_ATT_THR) != 0) {
        // reduce gains when flying at high speed in Q modes:

        // critical parameter: violent oscillations if too high
        // sudden loss of attitude control if too low
        const float min_scale = tailsitter.gain_scaling_min;
        float tthr = 1.25f * hover_throttle;

        // reduce control surface throws at large tilt
        // angles (assuming high airspeed)
        // ramp down from 1 to max_atten at tilt angles over trans_angle
        // (angles here are represented by their cosines)

        // Note that the cosf call will be necessary if trans_angle becomes a parameter
        // but the C language spec does not guarantee that trig functions can be used
        // in constant expressions, even though gcc currently allows it.
        constexpr float c_trans_angle = 0.9238795; // cosf(.125f * M_PI)

        // alpha = (1 - max_atten) / (c_trans_angle - cosf(radians(90)));
        const float alpha = (1 - min_scale) / c_trans_angle;
        const float beta = 1 - alpha * c_trans_angle;

        const float c_tilt = ahrs_view->get_rotation_body_to_ned().c.z;
        if (c_tilt < c_trans_angle) {
            spd_scaler = constrain_float(beta + alpha * c_tilt, min_scale, 1.0f);
            // reduce throttle attenuation threshold too
            tthr = 0.5f * hover_throttle;
        }
        // if throttle is above hover thrust, apply additional attenuation
        if (throttle > tthr) {
            const float throttle_atten = 1 - (throttle - tthr) / (1 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_float(spd_scaler, min_scale, 1.0f);
        }

        // limit positive and negative slew rates of applied speed scaling
        constexpr float posTC = 2.0f;   // seconds
        constexpr float negTC = 1.0f;   // seconds
        const float posdelta = plane.G_Dt / posTC;
        const float negdelta = plane.G_Dt / negTC;
        spd_scaler = constrain_float(spd_scaler, last_spd_scaler - negdelta, last_spd_scaler + posdelta);
        last_spd_scaler = spd_scaler;

        // also apply throttle scaling if enabled
        if ((spd_scaler >= 1.0f) && ((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0)) {
            spd_scaler = MAX(throttle_scaler,1.0f);
        }

    } else if (((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_DISK_THEORY) != 0) && is_positive(tailsitter.disk_loading.get())) {
        // Use disk theory to estimate the velocity over the control surfaces
        // https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html

        float airspeed;
        if (!ahrs.airspeed_estimate(airspeed)) {
            // No airspeed estimate, use throttle scaling
            spd_scaler = throttle_scaler;

        } else {


            // use the equation: T = 0.5 * rho * A (Ue^2 - U0^2) solved for Ue^2:
            // Ue^2 = (T / (0.5 * rho *A)) + U0^2
            // We don't know thrust or disk area, use T = (throttle/throttle_hover) * weight
            // ((t / t_h ) * weight) / (0.5 * rho * A) = ((t / t_h) * mass * 9.81) / (0.5 * rho * A)
            // (mass / A) is disk loading DL so:
            // Ue^2 = (((t / t_h) * DL * 9.81)/(0.5 * rho)) + U0^2

            const float rho = SSL_AIR_DENSITY * plane.barometer.get_air_density_ratio();
            float hover_rho = rho;
            if ((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
                // if applying altitude correction use sea level density for hover case
                hover_rho = SSL_AIR_DENSITY;
            }

            // hover case: (t / t_h) = 1 and U0 = 0
            const float sq_hover_outflow = (tailsitter.disk_loading.get() * GRAVITY_MSS) / (0.5f * hover_rho);


            // calculate the true outflow speed
            const float sq_outflow = (((throttle/hover_throttle) *  tailsitter.disk_loading.get() * GRAVITY_MSS) / (0.5f * rho)) + sq(MAX(airspeed,0));

            // Scale by the ratio of squared hover outflow velocity to squared actual outflow velocity
            spd_scaler = tailsitter.throttle_scale_max;
            if (is_positive(sq_outflow)) {
                spd_scaler = constrain_float(sq_hover_outflow / sq_outflow, tailsitter.gain_scaling_min.get(), tailsitter.throttle_scale_max.get());
            }
        }

    } else if ((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0) {
        spd_scaler = throttle_scaler;
    }

    if ((tailsitter.gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
        // air density correction
        spd_scaler /= plane.barometer.get_air_density_ratio();
    }

    // record for QTUN log
    log_spd_scaler = spd_scaler;

    const SRV_Channel::Aux_servo_function_t functions[] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_rudder,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        if ((functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft) || (functions[i] == SRV_Channel::Aux_servo_function_t::k_tiltMotorRight)) {
            // always apply throttle scaling to tilts
            v *= throttle_scaler;
        } else {
            v *= spd_scaler;
        }
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}
