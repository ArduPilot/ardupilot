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

#ifndef __BEV_EFFECTORS_H__
#define __BEV_EFFECTORS_H__

#include <RC_Channel.h>
#include <AP_MotorsY6.h>
#include <AP_Hal.h>

//BEV other effectors classes needed by the master effector class
#include "BEV_Gear.h"
#include "BEV_TransitionState.h"
#include "BEV_Motors.h"

/// @class      BEV_EFFECTORS
class BEV_Effectors : public BEV_Motors {
public:
    //BEV_Effectors( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw) : AP_MotorsY6(rc_roll, rc_pitch, rc_throttle, rc_yaw) {};
    //constructor

    BEV_Effectors(BEV_TransitionState::transition_callback_fn_t to_copter_callback, BEV_TransitionState::transition_callback_fn_t to_plane_callback, //transition callbacks
                  BEV_TransitionState::transition_callback_fn_t to_copternav_callback, //callback called at end of copternav suppression
                  const int16_t &channel_roll_out,const  int16_t &channel_pitch_out, const int16_t &channel_throttle_out, const int16_t &channel_rudder_out, //plane inputs
                  RC_Channel &rc_elevon_1, RC_Channel &rc_elevon_2, //plane outputs
                  RC_Channel &rc_roll, RC_Channel &rc_pitch, RC_Channel &rc_throttle, RC_Channel &rc_yaw, //copter
                  RC_Channel &rc_transition_switch, RC_Channel &rc_transition_out, //transition
                  RC_Channel &rc_gear_switch, RC_Channel &rc_gear_out_1); //gear

    virtual void Init();
    //for compliance with AP_Motors* parent classes
    //outputing is done slightly differently. output_armed only writes desired motors commands (front motors adjusted by transitionAngle)
    //to the _motors[] array. writing to the hal.rc_out object is actually done in the blend_all_effectors function. blend_all_effectors
    //must be called for outputs to be written.
    void output(bool radio_failsafe = false, int16_t target_pitch = 0);
    void output_armed();
    void output_disarmed();
    //gear interactions
    void gear_raise() {_gear.raise();}
    void gear_lower() {_gear.lower();}
    void gear_toggle() {_gear.toggle();}
    bool gear_is_raised() {return _gear.is_raised();}
    bool gear_is_lowered() {return _gear.is_lowered();}

    int8_t transition_get_direction() {return _transitionState.get_transition_direction();}
    float transition_get_angle_deg() {return _transitionState.get_transition_angle_deg();}
    void transition_to_copter() {_transitionState.to_copter();}
    void transition_to_plane() {_transitionState.to_plane();}
    void transition_toggle() {_transitionState.toggle();}
    bool transition_is_pitch_override() {return _transitionState.is_pitch_override();}
    int16_t transition_get_pitch_override() {return _transitionState.get_pitch_override();}
    bool transition_is_nav_suppressed() {return _transitionState.is_nav_suppressed();}
    bool transition_is_copter_active() {return _transitionState.is_copter_active();}
    bool transition_is_plane_active() {return _transitionState.is_plane_active();}
    bool transition_is_full_copter() {return _transitionState.is_full_copter();}
    bool transition_is_full_plane() {return _transitionState.is_full_plane();}
    bool transition_is_transitioning() {return _transitionState.is_transitioning();}
    uint8_t transition_get_copter_percent() {return _transitionState.get_copter_percent();}
    uint8_t transition_get_plane_percent() {return _transitionState.get_plane_percent();}

protected:
    void calc_motor_outputs(); //commands for the six Y6 motors. Does nothing if copter claw is not active. not called when disarmed
    void calc_servo_outputs(); //commands for the two elevons. Does nothing if plane claw is not active. Called when disarmed
    void blend_all_effectors(); //blends commands to all effectors using percentage of transition from BEV_TransitionState

private:
    //converts angle [-4500 4500] to output. RC_Channel probably does this too but it's too complicated and I can't figure out how to do it
    static void output_to_channel(RC_Channel & rc_out, int16_t angle);

    //keeps track of plane/copter switches
    BEV_TransitionState _transitionState;
    BEV_Gear _gear;

    //plane inputs
    const int16_t& _channel_roll_out;      //plane roll
    const int16_t& _channel_pitch_out;     //plane pitch
    const int16_t& _channel_throttle_out;  //plane throttle
    const int16_t& _channel_rudder_out;    //plane roll
    //plane outputs
    RC_Channel& _rc_elevon_1; //left elevon
    RC_Channel& _rc_elevon_2; //right elevon

    //copter controller outputs
    RC_Channel& _rc_roll; //copter roll, radio input
    RC_Channel& _rc_pitch; //copter pitch, radio input
    RC_Channel& _rc_throttle; //copter throttle, radio input
    RC_Channel& _rc_yaw; //copter yaw, radio input

    //information for blender
    int16_t _desired_plane_motors_out[AP_MOTORS_MAX_NUM_MOTORS];
    int16_t _desired_copter_servos_out[2];

};

#endif  // BEV_EFFECTORS
