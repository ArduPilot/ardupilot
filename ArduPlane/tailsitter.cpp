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
 */

#include "Plane.h"

/*
  return true when flying a tailsitter
 */
bool QuadPlane::is_tailsitter(void)
{
    return available() && frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER;
}

/*
  check if we are flying as a tailsitter
 */
bool QuadPlane::tailsitter_active(void)
{
    return is_tailsitter() && in_vtol_mode();
}

/*
  run output for tailsitters
 */
void QuadPlane::tailsitter_output(void)
{
    if (!is_tailsitter()) {
        return;
    }
    if (!tailsitter_active()) {
        if (tailsitter.vectored_forward_gain > 0) {
            // thrust vectoring in fixed wing flight
            float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
            float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
            float tilt_left  = (elevator + aileron) * tailsitter.vectored_forward_gain;
            float tilt_right = (elevator - aileron) * tailsitter.vectored_forward_gain;
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
        }
        return;
    }

    motors_output();
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    // scale factor required for normalizing roll/pitch/yaw servo values
    const float angle_servo_max = SERVO_MAX;
    const float throttle_servo_max = 100.0f;

    static float orig_pkP = -1.0f;
    static float orig_rkP = -1.0f;
    static float orig_ykP = -1.0f;

    if (tailsitter.tcomp_gain > 0) {
        if (orig_pkP < 0.0f) {
            orig_pkP = attitude_control->get_rate_pitch_pid().kP();
        }
        if (orig_rkP < 0.0f) {
            orig_rkP = attitude_control->get_rate_roll_pid().kP();
        }
        if (orig_ykP < 0.0f) {
            orig_ykP = attitude_control->get_rate_yaw_pid().kP();
        }

        float pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5f;
        float pitch_error_norm = constrain_float(fabsf(pitch_error_cd), 0, angle_servo_max) / angle_servo_max;

        float yaw_error_cd = (wrap_360_cd(attitude_control->get_att_target_euler_cd().z) - ahrs_view->yaw_sensor) * 0.5;
        float yaw_error_norm = constrain_float(fabsf(yaw_error_cd), 0, angle_servo_max) / angle_servo_max;

        float roll_error_cd = (plane.nav_roll_cd - ahrs_view->roll_sensor) * 0.5f;
        float roll_error_norm = constrain_float(fabsf(roll_error_cd), 0, angle_servo_max) / angle_servo_max;

        float norm_err_max = 0.0f;
        float thr_boost = 0.0f;
        float elev_d = 0.0f;
        float ail_scale = 0.0f;

        // assume that k_throttle will not be assigned for twin_engine tailsitters
        bool twin_engine = !SRV_Channels::function_assigned(SRV_Channel::k_throttle);

        // assume that k_tiltMotorLeft will be assigned for tilt-vectored dual-motor vehicles
        bool vectored_thrust = SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft);

        // allow throttle boost only if not tilt-vectored and throttle stick is above 10%
        bool enable_boost = !vectored_thrust && plane.channel_throttle->get_control_in() > 10;

        float avg_thr_norm = 0;
        if (twin_engine) {
            // boost throttle when pitch or yaw error is large (roll is controlled with diff. throttle)
            norm_err_max = fmaxf(yaw_error_norm, pitch_error_norm);
            thr_boost = 1.0f + norm_err_max * tailsitter.tcomp_gain;

            // prioritize pitch over yaw by attenuating aileron control when pitch demand is large
            // Apparently get_output_norm will return zero if k_elevator is not mapped to a PWM channel???
            elev_d = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / angle_servo_max;

            // use an exponential mapping to attenuate only for larger values of elev_d
            float elevd2 = elev_d * elev_d;
            float elevd4 = elevd2 * elevd2;
            ail_scale = 1.0f - (0.5f * elevd4);

            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,
                    ail_scale * SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));

            int16_t thr_scaledL = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
            int16_t thr_scaledR = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);

            // boost throttle only if throttle stick is above 10%
            if (enable_boost) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, thr_boost * thr_scaledL);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, thr_boost * thr_scaledR);
                avg_thr_norm = thr_boost * (thr_scaledL + thr_scaledR) / (2 * throttle_servo_max);
            } else {
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, thr_scaledL);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, thr_scaledR);
                avg_thr_norm = (thr_scaledL + thr_scaledR) / (2 * throttle_servo_max);
            }


        } else {
            // boost throttle when roll error is large (rudder sometimes has insufficient authority at low throttle)
            norm_err_max = roll_error_norm;

            int16_t thr_scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

            if (enable_boost && norm_err_max > 0.1f) {
                // scaling throttle doesn't work in QHOVER, because it's canceled by the altitude controller
                // instead, just add 0.5 (approx. hover throttle) to the scaled error
                thr_boost = 0.5f + norm_err_max * tailsitter.tcomp_gain;
                avg_thr_norm = thr_boost;

                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, avg_thr_norm * throttle_servo_max);
            } else {
                avg_thr_norm = thr_scaled / throttle_servo_max;
            }
        }

        // reduce PID gains in proportion to throttle level
        float pitch_kP = orig_pkP;
        float roll_kP = orig_rkP;
        float yaw_kP = orig_rkP;

        float gain_scale = 1.0;
        if (true && avg_thr_norm > 0.5f) { //motors->get_throttle_hover()) {
            gain_scale = 0.5f / avg_thr_norm;
            pitch_kP *= gain_scale;
            roll_kP *= gain_scale;
            yaw_kP *= gain_scale;
        }
        attitude_control->get_rate_pitch_pid().kP(pitch_kP);
        attitude_control->get_rate_roll_pid().kP(roll_kP);
        attitude_control->get_rate_yaw_pid().kP(yaw_kP);

        // temporary debug logging
        static int dec_count=0;
        if ((dec_count++ >= 3) && (thr_boost > 1.05f)) {
            dec_count = 0;
//            hal.console->printf("roll_error_norm: %5.3f, thr_boosted: %5.3f, thr_scaled: %5.3f\n",
//                    (double) roll_error_norm,
//                    (double) thr_boosted,
//                    (double) thr_scaled);
            DataFlash_Class::instance()->Log_Write("TCOM", "TimeUS,Perr,YErr,ThrA,ElvD,AilS,PkP,DesY,Yaw,Gscl", "Qfffffffff",
                                                   AP_HAL::micros64(),
                                                   (double) pitch_error_norm,
                                                   (double) yaw_error_norm,
                                                   (double) avg_thr_norm,
                                                   (double) elev_d,
                                                   (double) ail_scale,
                                                   (double) pitch_kP,
                                                   (double) wrap_360_cd(attitude_control->get_att_target_euler_cd().z),
                                                   (double) ahrs_view->yaw_sensor,
                                                   (double) gain_scale
                                                   );
        }

    }

    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -4500, 4500) / 4500.0;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * 4500;
        float tilt_left  = extra_elevator + (elevator + aileron) * tailsitter.vectored_hover_gain;
        float tilt_right = extra_elevator + (elevator - aileron) * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= 4500 || fabsf(tilt_right) >= 4500) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    }
    
    
    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        hal.rcin->read(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.channel_throttle->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle*100 ||
        labs(ahrs_view->roll_sensor) > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > 2000) {
        return true;
    }
    // still waiting
    return false;
}

// handle different tailsitter input types
void QuadPlane::tailsitter_check_input(void)
{
    if (tailsitter_active() &&
        tailsitter.input_type == TAILSITTER_INPUT_PLANE) {
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
