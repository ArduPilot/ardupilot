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

    const float servo_scale_magic = 4500.0f;
    static float orig_kP = -1.0f;

    if (tailsitter.tcomp_gain > 0) {
        if (orig_kP < 0.0f) {
            orig_kP = attitude_control->get_rate_pitch_pid().kP();
        }
        // boost throttle when pitch or yaw error is large
        float pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5f;
        float pitch_error_norm = constrain_float(fabsf(pitch_error_cd), 0, servo_scale_magic) / servo_scale_magic;

        float yaw_error_cd = (wrap_360_cd(attitude_control->get_att_target_euler_cd().z) - ahrs_view->yaw_sensor) * 0.5;
        float yaw_error_norm = constrain_float(fabsf(yaw_error_cd), 0, servo_scale_magic) / servo_scale_magic;

        // prioritize pitch over yaw by attenuating aileron control when pitch demand is large
        // Apparently get_output_norm will return zero if k_elevator is not mapped to a PWM channel???
        float elev_d = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / servo_scale_magic;
        // use an exponential mapping to attenuate only for larger values of elev_d
        float elevd2 = elev_d * elev_d;
        float elevd4 = elevd2 * elevd2;
        float ail_scale = 1.0f - (0.5f * elevd4);

        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,
                ail_scale * SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));

        float norm_err_max = fmaxf(yaw_error_norm, pitch_error_norm);
        float thr_boost = 1.0f + norm_err_max * tailsitter.tcomp_gain;

        int16_t thr_scaledL = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleLeft);
        int16_t thr_scaledR = SRV_Channels::get_output_scaled(SRV_Channel::k_throttleRight);

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, thr_boost * thr_scaledL);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, thr_boost * thr_scaledR);

        // reduce pitch PID gains in proportion to throttle level
        float avg_thr_norm = (thr_scaledL + thr_scaledR) / 200.0f;
        float gain_scale = fminf(1.0f, 0.5f / avg_thr_norm);
        float pitch_kP = gain_scale * orig_kP;
        attitude_control->get_rate_pitch_pid().kP(pitch_kP);

        // temporary debug logging
        static int dec_count=0;
        if (dec_count++ >= 3) {
            dec_count = 0;
            hal.console->printf("target_yaw: %5.3f, yaw: %5.3f\n",
                    (double) attitude_control->get_att_target_euler_cd().z,
                    (double) ahrs_view->yaw_sensor);
            DataFlash_Class::instance()->Log_Write("TCOM", "TimeUS,Perr,YErr,ThrB,ElvD,AilS,PkP,DesY,Yaw,Gscl", "Qfffffffff",
                                                   AP_HAL::micros64(),
                                                   (double) pitch_error_norm,
                                                   (double) yaw_error_norm,
                                                   (double) thr_boost,
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
