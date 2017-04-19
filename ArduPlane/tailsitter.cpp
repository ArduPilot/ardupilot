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

    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        float tilt_left  = (elevator + aileron) * tailsitter.vectored_hover_gain;
        float tilt_right = (elevator - aileron) * tailsitter.vectored_hover_gain;
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
