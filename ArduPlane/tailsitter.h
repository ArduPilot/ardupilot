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
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Motors/AP_MotorsTailsitter.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tailsitter_Transition;
class Tailsitter
{
friend class QuadPlane;
friend class Plane;
public:

    Tailsitter(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    // return true when flying a control surface only tailsitter
    bool is_control_surface_tailsitter(void) const;

    // return true when flying a tailsitter in VTOL
    bool active(void);
    
    // create outputs for tailsitters
    void output(void);

    // handle different tailsitter input types
    void check_input(void);

    // check if we have completed transition to fixed wing
    bool transition_fw_complete(void);

    // return true if we are a tailsitter in FW flight
    bool is_in_fw_flight(void) const;

    // check if we have completed transition to vtol
    bool transition_vtol_complete(void) const;

    // return true if transition to VTOL flight
    bool in_vtol_transition(uint32_t now = 0) const;

    // account for control surface speed scaling in VTOL modes
    void speed_scaling(void);

    // return the transition_angle_vtol value
    int8_t get_transition_angle_vtol() const;

    // return true if pitch control should be relaxed
    bool relax_pitch();

    // tailsitter speed scaler
    float last_spd_scaler = 1.0f; // used to slew rate limiting with TAILSITTER_GSCL_ATT_THR option
    float log_spd_scaler; // for QTUN log

    static const struct AP_Param::GroupInfo var_info[];

    // bit 0 enables plane mode and bit 1 enables body-frame roll mode
    enum input {
        TAILSITTER_INPUT_PLANE   = (1U<<0),
        TAILSITTER_INPUT_BF_ROLL = (1U<<1)
    };

    enum gscl_mask {
        TAILSITTER_GSCL_THROTTLE = (1U<<0),
        TAILSITTER_GSCL_ATT_THR = (1U<<1),
        TAILSITTER_GSCL_DISK_THEORY = (1U<<2),
        TAILSITTER_GSCL_ALTITUDE = (1U<<3),
    };

    AP_Int8 enable;
    AP_Int8 transition_angle_fw;
    AP_Float transition_rate_fw;
    AP_Int8 transition_angle_vtol;
    AP_Float transition_rate_vtol;
    AP_Float transition_throttle_vtol;
    AP_Int8 input_type;
    AP_Float vectored_forward_gain;
    AP_Float vectored_hover_gain;
    AP_Float vectored_hover_power;
    AP_Float throttle_scale_max;
    AP_Float gain_scaling_min;
    AP_Float max_roll_angle;
    AP_Int16 motor_mask;
    AP_Float scaling_speed_min;
    AP_Float scaling_speed_max;
    AP_Int16 gain_scaling_mask;
    AP_Float disk_loading;
    AP_Float VTOL_roll_scale;
    AP_Float VTOL_pitch_scale;
    AP_Float VTOL_yaw_scale;
    AP_Float disk_loading_min_outflow;

    AP_MotorsTailsitter* tailsitter_motors;

private:

    bool setup_complete;

    // true when flying a tilt-vectored tailsitter
    bool _is_vectored;

    // true is outputs are configured
    bool _have_elevator;
    bool _have_aileron;
    bool _have_rudder;

    // refences for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    // transition logic
    Tailsitter_Transition* transition;

};


// Transition for tailsitters
class Tailsitter_Transition : public Transition
{
friend class Tailsitter;
public:

    Tailsitter_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tailsitter& _tailsitter):Transition(_quadplane, _motors), tailsitter(_tailsitter) {};

    void update() override;

    void VTOL_update() override;

    void force_transition_complete() override;

    bool complete() const override { return transition_state == TRANSITION_DONE; }

    // setup for the transition back to fixed wing
    void restart() override;

    uint8_t get_log_transition_state() const override { return static_cast<uint8_t>(transition_state); }

    bool active() const override { return transition_state != TRANSITION_DONE; }

    bool show_vtol_view() const override;

    void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd, bool& allow_stick_mixing) override;

    MAV_VTOL_STATE get_mav_vtol_state() const override;

    bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) override;

    bool allow_weathervane() override;

private:

    enum {
        TRANSITION_ANGLE_WAIT_FW,
        TRANSITION_ANGLE_WAIT_VTOL,
        TRANSITION_DONE
    } transition_state;

    // for transition to VTOL flight
    uint32_t vtol_transition_start_ms;
    float vtol_transition_initial_pitch;

    // for rate limit of VTOL flight
    uint32_t vtol_limit_start_ms;
    float vtol_limit_initial_pitch;

    // for rate limit of FW flight
    uint32_t fw_limit_start_ms;
    float fw_limit_initial_pitch;

    // for transition to FW flight
    uint32_t fw_transition_start_ms;
    float fw_transition_initial_pitch;

    // time when we were last in a vtol control mode
    uint32_t last_vtol_mode_ms;

    Tailsitter& tailsitter;

};
