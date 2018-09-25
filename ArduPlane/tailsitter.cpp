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

#include "Plane.h"

/*
  return true when flying a tailsitter
 */
bool QuadPlane::is_tailsitter(void) const
{
    return available() && ((frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) || 
                           (tailsitter.motor_mask != 0));
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
    if (!is_tailsitter()) {
        return;
    }

    // record plane outputs in case they are needed for interpolation
    float fw_aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float fw_elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    float fw_rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);

     // thrust vectoring in fixed wing flight
    float fw_tilt_left = 0;
    float fw_tilt_right = 0;
    if (tailsitter.vectored_forward_gain > 0) {
        fw_tilt_left = (fw_elevator + fw_aileron) * tailsitter.vectored_forward_gain;
        fw_tilt_right = (fw_elevator - fw_aileron) * tailsitter.vectored_forward_gain;
    }

    if ((!tailsitter_active() || in_tailsitter_vtol_transition()) && !assisted_flight) {
        // output tilts for forward flight
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, fw_tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, fw_tilt_right);

         // get FW controller throttle demand and mask of motors enabled during forward flight
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        if (in_tailsitter_vtol_transition() && !throttle_wait && is_flying() && hal.util->get_soft_armed()) {
            /*
              during transitions to vtol mode set the throttle to
              hover thrust, center the rudder and set the altitude controller
              integrator to the same throttle level
             */
            throttle = motors->get_throttle_hover() * 100;
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);

            // override AP_MotorsTailsitter throttles during back transition
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle);
        }
        if (tailsitter.motor_mask != 0) {
            // set AP_MotorsMatrix throttles enabled for forward flight
            motors->output_motor_mask(throttle * 0.01f, tailsitter.motor_mask, plane.rudder_dt);
        }
        return;
    }

    if (assisted_flight) {
        control_stabilize();
        motors_output(true);
    } else {
        motors_output(false);
    }

    // if in Q assist still a good idea to use copter I term and zero plane I to prevent windup
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    // pull in copter control outputs
    float aileron = motors->get_yaw()*-SERVO_MAX;
    float elevator = motors->get_pitch()*SERVO_MAX;
    float rudder = motors->get_roll()*SERVO_MAX;
    float throttle = motors->get_throttle() * 100;
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    // scale surfaces for throttle
    if (hal.util->get_soft_armed()) {
        float scaling = get_tailsitter_speed_scaling();
        rudder = constrain_float(rudder * scaling, -SERVO_MAX, SERVO_MAX);
        aileron = constrain_float(aileron * scaling, -SERVO_MAX, SERVO_MAX);
        elevator = constrain_float(elevator * scaling, -SERVO_MAX, SERVO_MAX);
        tilt_left = constrain_float(tilt_left * scaling, -SERVO_MAX, SERVO_MAX);
        tilt_right = constrain_float(tilt_right * scaling, -SERVO_MAX, SERVO_MAX);
    }

    if (tailsitter.vectored_hover_gain > 0) {
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        float extra_sign = extra_pitch > 0?1:-1;
        // only apply extra elevator in Q modes
        float extra_elevator = 0.0f;
        if (!assisted_flight) {
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * SERVO_MAX;
        }
        tilt_left  = extra_elevator + tilt_left * tailsitter.vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= SERVO_MAX || fabsf(tilt_right) >= SERVO_MAX) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
    }

    // apply speed scaling to interpolate between fixed wing and VTOL outputs based on airspeed
    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);
    const float scaling_range = tailsitter.scaling_speed_max - tailsitter.scaling_speed_min;
    // only bother if it will change the output
    if (aspeed > tailsitter.scaling_speed_min && have_airspeed && !is_zero(scaling_range)) {

        // match the Q rates with plane controller
        if (!assisted_flight) {
            // no fixed wing yaw controller so cannot stabilize VTOL roll
            const float pitch_rate = attitude_control->get_rate_pitch_pid().get_pid_info().desired * 100;
            const float yaw_rate = attitude_control->get_rate_yaw_pid().get_pid_info().desired * 100;
            const float speed_scaler = plane.get_speed_scaler();

            // due to reference frame change roll and yaw are swapped, use roll as rudder input and output direct as with plane
            fw_aileron = plane.rollController.get_rate_out(-yaw_rate, speed_scaler);
            fw_elevator = plane.pitchController.get_rate_out(pitch_rate, speed_scaler);
            fw_rudder = plane.channel_roll->get_control_in();
        }

        // calculate ratio of gains
        float fw_ratio = (aspeed - tailsitter.scaling_speed_min) / scaling_range;
        fw_ratio = constrain_float(fw_ratio, 0.0f, 1.0f);
        const float VTOL_rato = 1.0f - fw_ratio;

        // calculate interpolated outputs
        aileron = aileron * VTOL_rato + fw_aileron * fw_ratio;
        elevator = elevator * VTOL_rato + fw_elevator * fw_ratio;
        rudder = rudder * VTOL_rato + fw_rudder * fw_ratio;
        tilt_left = tilt_left * VTOL_rato + fw_tilt_left * fw_ratio;
        tilt_right = tilt_right * VTOL_rato + fw_tilt_right * fw_ratio;
    }

    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            aileron = plane.channel_roll->get_control_in_zero_dz();
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            elevator = plane.channel_pitch->get_control_in_zero_dz();
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            throttle = plane.get_throttle_input(true);
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            rudder = plane.channel_rudder->get_control_in_zero_dz();
        }
    }

    // set outputs
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_fw_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    int32_t roll_cd = labs(ahrs_view->roll_sensor);
    if (roll_cd > 9000) {
        roll_cd = 18000 - roll_cd;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle*100 ||
        roll_cd > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > uint32_t(transition_time_ms)) {
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
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    if (labs(plane.ahrs.pitch_sensor) > tailsitter.transition_angle*100 ||
        labs(plane.ahrs.roll_sensor) > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > 2000) {
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
        (tailsitter.input_type == TAILSITTER_INPUT_BF_ROLL ||
         tailsitter.input_type == TAILSITTER_INPUT_PLANE)) {
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
bool QuadPlane::in_tailsitter_vtol_transition(void) const
{
    return is_tailsitter() && in_vtol_mode() && transition_state == TRANSITION_ANGLE_WAIT_VTOL;
}

/*
  calculate speed scaler of control surfaces in VTOL modes
*/
float QuadPlane::get_tailsitter_speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float spd_scaler = 1;

    // If throttle_scale_max is => 1, boost gains at low throttle
    if (tailsitter.throttle_scale_max >= 1) {
        if (is_zero(throttle)) {
            spd_scaler = tailsitter.throttle_scale_max;
        } else {
            spd_scaler = constrain_float(hover_throttle / throttle, 0, tailsitter.throttle_scale_max);
        }
        return spd_scaler;

    } else {
        // reduce gains when flying at high speed in Q modes:

        // critical parameter: violent oscillations if too high
        // sudden loss of attitude control if too low
        constexpr float max_atten = 0.2f;
        float tthr = 1.25f * hover_throttle;
        float aspeed;
        bool airspeed_enabled = ahrs.airspeed_sensor_enabled();

        // If there is an airspeed sensor use the measured airspeed
        // The airspeed estimate based only on GPS and (estimated) wind is
        // not sufficiently accurate for tailsitters.
        // (based on tests in RealFlight 8 with 10kph wind)
        if (airspeed_enabled && ahrs.airspeed_estimate(&aspeed)) {
            // plane.get_speed_scaler() doesn't work well for copter tailsitters
            // ramp down from 1 to max_atten as speed increases to airspeed_max
            spd_scaler = constrain_float(1 - (aspeed / plane.aparm.airspeed_max), max_atten, 1.0f);
        } else {
            // if no airspeed sensor reduce control surface throws at large tilt
            // angles (assuming high airspeed)

            // ramp down from 1 to max_atten at tilt angles over trans_angle
            // (angles here are represented by their cosines)
            constexpr float c_trans_angle = cosf(.125f * M_PI);
            constexpr float alpha = (1 - max_atten) / (c_trans_angle - cosf(radians(90)));
            constexpr float beta = 1 - alpha * c_trans_angle;
            const float c_tilt = ahrs_view->get_rotation_body_to_ned().c.z;
            if (c_tilt < c_trans_angle) {
                spd_scaler = constrain_float(beta + alpha * c_tilt, max_atten, 1.0f);
                // reduce throttle attenuation threshold too
                tthr = 0.5f * hover_throttle;
            }
        }
        // if throttle is above hover thrust, apply additional attenuation
        if (throttle > tthr) {
            const float throttle_atten = 1 - (throttle - tthr) / (1 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_float(spd_scaler, max_atten, 1.0f);
        }

        // limit positive and negative slew rates of applied speed scaling
        constexpr float posTC = 5.0f;   // seconds
        constexpr float negTC = 2.0f;   // seconds
        const float posdelta = plane.G_Dt / posTC;
        const float negdelta = plane.G_Dt / negTC;
        static float last_scale = 0;
        const float scale = constrain_float(spd_scaler, last_scale - negdelta, last_scale + posdelta);
        last_scale = scale;
        return scale;
    }
}
