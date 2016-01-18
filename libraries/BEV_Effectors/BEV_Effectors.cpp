/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>
#include "BEV_Effectors.h"

extern const AP_HAL::HAL& hal;

//
// public methods
//

BEV_Effectors::BEV_Effectors(BEV_TransitionState::transition_callback_fn_t to_copter_callback, BEV_TransitionState::transition_callback_fn_t to_plane_callback, //transition callbacks
        BEV_TransitionState::transition_callback_fn_t to_copternav_callback, //callback called at end of copternav suppression
                             const int16_t &channel_roll_out, const int16_t &channel_pitch_out, const int16_t &channel_throttle_out, const int16_t &channel_rudder_out,
                             RC_Channel &rc_elevon_1, RC_Channel &rc_elevon_2,
                             RC_Channel &rc_roll, RC_Channel &rc_pitch, RC_Channel &rc_throttle, RC_Channel &rc_yaw,
                             RC_Channel &rc_transition_switch, RC_Channel &rc_transition_out,
                             RC_Channel &rc_gear_switch, RC_Channel & rc_gear_out_1)
                           : BEV_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw), //parent class
                             _transitionState(rc_transition_switch, rc_transition_out, to_copter_callback, to_plane_callback, to_copternav_callback),
                             _gear(rc_gear_switch, rc_gear_out_1),
                             _channel_roll_out(channel_roll_out),
                             _channel_pitch_out(channel_pitch_out),
                             _channel_throttle_out(channel_throttle_out),
                             _channel_rudder_out(channel_rudder_out),
                             _rc_elevon_1(rc_elevon_1),
                             _rc_elevon_2(rc_elevon_2),
                             _rc_roll(rc_roll),
                             _rc_pitch(rc_pitch),
                             _rc_throttle(rc_throttle),
                             _rc_yaw(rc_yaw)
{
    ;
}


void BEV_Effectors::Init()
{
    //because of the stupid motors test this function can be called multiple times. Counter here to keep this from being a problem.
    static bool first_call = true;
    if(first_call) {
        first_call = false;
    } else {
        return;
    }

    //parent class init
    BEV_Motors::Init();

    //init gear and transition
    _gear.Init();
    _transitionState.Init();

    //center the elevons
    _rc_elevon_1.radio_out = _rc_elevon_1.radio_trim;
    _rc_elevon_1.enable_out();
    _rc_elevon_2.radio_out = _rc_elevon_2.radio_trim;
    _rc_elevon_2.enable_out();
}
void BEV_Effectors::output(bool radio_failsafe, int16_t target_pitch)
{
    _gear.update(radio_failsafe);
    _transitionState.update(radio_failsafe, target_pitch, _gear.is_raised());

    BEV_Motors::set_transition_angle_deg(_transitionState.get_transition_angle_deg());

    //calculate desired copter motors
    if(_transitionState.is_copter_active()) {
        BEV_Motors::output();
    }

    calc_servo_outputs();
    calc_motor_outputs();
    blend_all_effectors();
}

void BEV_Effectors:: output_armed()
{
    BEV_Motors::output_armed();
}

void BEV_Effectors::output_disarmed()
{
    //calls output_min
    BEV_Motors::output_disarmed();
}

//
// protected methods
//

//plane commands for the six Y6 motors. Does nothing if plane claw is not active.
//copter motors called in BEV_Motors::output_armed
void BEV_Effectors::calc_motor_outputs()
{
    //channel_throttle_out is [0 1000]
    //valid range for motors is in BEV_Motors class. Can be backed out here
    int16_t out_min_pwm = _rc_throttle.radio_min + _spin_when_armed_ramped;      // minimum pwm value we can send to the motors
    int16_t out_max_pwm = _rc_throttle.radio_max;                      // maximum pwm value we can send to the motors
    int16_t out_delta_pwm = out_max_pwm - out_min_pwm;

    //no calculations needed if plane is not currently active
    if(!_transitionState.is_plane_active()) {
        return;
    }

    //are we armed
    if(_flags.armed){
        //scale the throttle command from [0 1000] to [0 out_delta_pwm].
        int16_t thrust_component = constrain_int16(_channel_throttle_out, 0, 1000)*out_delta_pwm/float(1000);

        //scale the rudder command from [-4500 4500] to [-out_delta_pwm/8 out_delta_pwm/8]. This gives it quarter authority
        int16_t rudder_component = (constrain_int16(_channel_rudder_out, -4500, 4500)/double(9000))*(out_delta_pwm/2);
        //figure out how much yaw can fit onto the current thrust_component without saturating a motor output
        int16_t yaw_max = min(out_max_pwm - thrust_component - out_min_pwm, thrust_component);
        //constrain the allowed yaw output to what can fit
        rudder_component = constrain_int16(rudder_component, -yaw_max, yaw_max);

        //calculating desired plane outputs for each motor. Motors are on channels 2-7
        _desired_plane_motors_out[2] = _rc_throttle.radio_min; //aft motors off
        _desired_plane_motors_out[3] = constrain_int16(thrust_component-rudder_component,0,out_delta_pwm) + out_min_pwm;
        _desired_plane_motors_out[4] = constrain_int16(thrust_component+rudder_component,0,out_delta_pwm) + out_min_pwm;
        _desired_plane_motors_out[5] =  _rc_throttle.radio_min; //aft motors off
        _desired_plane_motors_out[6] = constrain_int16(thrust_component-rudder_component,0,out_delta_pwm) + out_min_pwm;
        _desired_plane_motors_out[7] = constrain_int16(thrust_component+rudder_component,0,out_delta_pwm) + out_min_pwm;
    } else {
        for(uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            _desired_plane_motors_out[i] = _rc_throttle.radio_min;
        }
    }

}
//commands for the two elevons. Does nothing if plane claw is not active. Called when disarmed
void BEV_Effectors::calc_servo_outputs()
{
    //desired copter servos out
    _desired_copter_servos_out[0] = 0; //0 degrees is neutral
    _desired_copter_servos_out[1] = 0;
}

//blends commands to all effectors using percentage of transition from BEV_TransitionState
//it is very important not to clobber _channel_*_out, blend_all_effectors may not be called immediately following setting the channel_*_out  variables
void BEV_Effectors::blend_all_effectors()
{
    static int16_t actual_servos_out[2];

    //servo outputs
    actual_servos_out[0] = _channel_roll_out;
    actual_servos_out[1] = _channel_pitch_out;

    //blending the axis commands to flaparon outputs. Roll and pitch can saturate one another. TO undo this, set the multiplier to 0.5. FOr better authority, leave multiplier at 0.75
    int16_t temp1 = (actual_servos_out[0] - actual_servos_out[1])*0.5;
    int16_t temp2 = (actual_servos_out[0] + actual_servos_out[1])*0.5;
    actual_servos_out[0] = constrain_int16(temp1, -4500, 4500);
    actual_servos_out[1] = constrain_int16(temp2, -4500, 4500);

    //blending motor outputs
    for(uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _motor_out[i] = 0.01*(_motor_out[i]*_transitionState.get_copter_percent() + _desired_plane_motors_out[i]*(100-_transitionState.get_copter_percent()));
    }

    //outputting to servos
    output_to_channel(_rc_elevon_1, actual_servos_out[0]);
    output_to_channel(_rc_elevon_2, actual_servos_out[1]);

    //outputting to motors on channels 2-7
    hal.rcout->write(2,_motor_out[2]);
    hal.rcout->write(3,_motor_out[3]);
    hal.rcout->write(4,_motor_out[4]);
    hal.rcout->write(5,_motor_out[5]);
    hal.rcout->write(6,_motor_out[6]);
    hal.rcout->write(7,_motor_out[7]);
}

//this function is here because the RC_Channel class makes this surprisingly difficult to accomplish
void BEV_Effectors::output_to_channel(RC_Channel &rc_out, int16_t angle)
{
    //deal with reversing the servo
    if(rc_out.get_reverse()) {
        angle = -angle;
    }

    //scale the desired output to meet the min and max set by the parameters
    if(angle>0) {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_max-rc_out.radio_trim)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    } else {
        rc_out.radio_out = constrain_int16( (float(angle)/4500)*(rc_out.radio_trim-rc_out.radio_min)+rc_out.radio_trim, rc_out.radio_min, rc_out.radio_max);
    }

    //actually write to the output pin
    rc_out.output();
}
