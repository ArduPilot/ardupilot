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
#define DEBUG_SPEED_SCALING 1

#if DEBUG_SPEED_SCALING
// first order IIR high pass filter
// coefficients b0 = b, b1 = -b
// equation: y_n = b0 x_n + b1 x_{n-1} - a y_{n-1}
// given b1 = -b0 = b: y_n = b (x_n - x_{n-1} - a y_{n-1}
// x1 is the new input value, y is the filtered result
// x0 is used to save the previous input
inline void high_pass(float x1, float &x0, float &y) {
    // coefficients for nyquist/30 cutoff:
    const float b = 0.974482;
    const float a = -0.948964;
    y = b * (x1 - x0) - a * y;
    x0 = x1;
}

// log speed scaler and highpass filtered gyro rates for validation
void log_CTHP(float spd_scaler, float VTOL_ratio, Vector3f rates) {

    // detect oscillation by monitoring gyros
    static float in0[3] = {0};
    static float hpOut[3] = {0};

    // highpass filter each gyro output
    high_pass(rates.x, in0[0], hpOut[0]);
    high_pass(rates.y, in0[1], hpOut[1]);
    high_pass(rates.z, in0[2], hpOut[2]);

    // square then lowpass filter the highpass outputs
    constexpr float lpcoef = 0.02f;
    static float msq[3] = {0};

    for (int i=0; i<3; i++) {
        msq[i] = (1 - lpcoef) * msq[i] +lpcoef * sq(hpOut[i]);
    }

    static uint32_t last_log_time = 0;
    uint32_t now = AP_HAL::millis();
    if ((now - last_log_time) >= (1000 / 100)) {
        last_log_time = now;

        AP::logger().Write("CTHP", "TimeUS,MSQr,MSQp,MSQy,HPr,HPp,HPy,Scl,SclV", "Qffffffff",
                            AP_HAL::micros64(),
                            msq[0], msq[1], msq[2],
                            hpOut[0], hpOut[1], hpOut[2],
                            spd_scaler, VTOL_ratio);
    }
}
#endif

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

void QuadPlane::scale_control_surfaces(float& fw_aileron, float& fw_elevator, float& fw_rudder, float& aileron, float& elevator, float& rudder){
    float VTOL_ratio = 1.0f;
    if (tailsitter.gain_scaling_mask & TAILSITTER_GSCL_INTERP) {
        float aspeed;
        bool have_airspeed = ahrs.airspeed_estimate(&aspeed);
        const float scaling_range = tailsitter.scaling_speed_max - tailsitter.scaling_speed_min;
        if (aspeed > tailsitter.scaling_speed_min && have_airspeed && !is_zero(scaling_range)) {
            // apply surface scaling to interpolate between fixed wing and VTOL outputs based on airspeed
            // kd0aij note: that the airspeed estimate based only on GPS and (estimated) wind is
            // not sufficiently accurate for tailsitters. (based on tests in RealFlight 8 with 10kph wind)

            if (!assisted_flight) {
                // match the Q rates with plane controller
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
            VTOL_ratio = 1.0f - fw_ratio;

            // calculate interpolated outputs
            aileron = aileron * VTOL_ratio + fw_aileron * fw_ratio;
            elevator = elevator * VTOL_ratio + fw_elevator * fw_ratio;
            rudder = rudder * VTOL_ratio + fw_rudder * fw_ratio;
        }
    }

    // scale surface throws using throttle and attitude
    float scaling = get_thr_att_gain_scaling();
    rudder = constrain_float(rudder * scaling, -SERVO_MAX, SERVO_MAX);
    aileron = constrain_float(aileron * scaling, -SERVO_MAX, SERVO_MAX);
    elevator = constrain_float(elevator * scaling, -SERVO_MAX, SERVO_MAX);

    log_CTHP(scaling, VTOL_ratio, ahrs_view->get_gyro_latest());
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
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

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

    if (assisted_flight && tailsitter_transition_fw_complete()) {
        hold_stabilize(throttle * 0.01f);
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
    throttle = motors->get_throttle() * 100;
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (tailsitter.vectored_hover_gain > 0) {
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
        if (!is_zero(extra_pitch)) {
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

    scale_control_surfaces(fw_aileron, fw_elevator, fw_rudder, aileron, elevator, rudder);

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

    if (hal.util->get_soft_armed()) {
        // set outputs
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }
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
        (tailsitter.input_type == TAILSITTER_INPUT_BF_ROLL_M ||
         tailsitter.input_type == TAILSITTER_INPUT_BF_ROLL_P ||
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
float QuadPlane::get_thr_att_gain_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float spd_scaler = 1.0f;

    if (tailsitter.gain_scaling_mask & TAILSITTER_GSCL_ATT_THR) {
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
    }

    // if gain attenuation isn't active and boost is enabled
    if ((spd_scaler >= 1.0f) && (tailsitter.gain_scaling_mask & TAILSITTER_GSCL_BOOST)) {
        // boost gains at low throttle
        if (is_zero(throttle)) {
            spd_scaler = tailsitter.throttle_scale_max;
        } else {
            spd_scaler = constrain_float(hover_throttle / throttle, 1.0f, tailsitter.throttle_scale_max);
        }
    }

    return spd_scaler;
}
