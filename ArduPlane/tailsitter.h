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

#include "quadplane.h"

/*
  tailsitter specific functionality
 */
class tailsitter : public QuadPlane {

public:

    tailsitter(AP_AHRS_NavEKF &_ahrs) :
        QuadPlane(_ahrs)
    {
    };

    bool setup() override;

    // use multicopter rate controller
    void multicopter_attitude_rate_update(float yaw_rate_cds) override;

    void hold_stabilize(float throttle_in) override;

    void init_z_target() override {
        // tailsitters gain lots of vertical speed in transisison, set target to the stopping point
        pos_control->set_target_to_stopping_point_z();
    };

    void get_acro_target_roll_yaw(float &target_roll, float &target_yaw) override;

    // vtol help for is_flying()
    bool is_flying(void) override;

    bool assistance_needed(float aspeed, bool have_airspeed) override;

    // update transition handling
    void update_transition(void) override;

    // update transition handling
    void update(void) override;

    // return true when tailsitter frame configured
    bool is_tailsitter(void) const override {return available();}

    // return true when flying a control surface only tailsitter tailsitter
    bool is_contol_surface_tailsitter(void) const;

    // return true when flying a tailsitter in VTOL
    bool tailsitter_active(void) const override;
    
    // create outputs for tailsitters
    void output(void) override;

    // handle different tailsitter input types
    void check_input(void) override;
    
    // check if we have completed transition to fixed wing
    bool tailsitter_transition_fw_complete(void);

    // return true if we are a tailsitter in FW flight
    bool is_tailsitter_in_fw_flight(void) const override;

    // check if we have completed transition to vtol
    bool tailsitter_transition_vtol_complete(void) const;

    bool show_vtol_view() const override;

    // account for control surface speed scaling in VTOL modes
    void tailsitter_speed_scaling(void);

    /*
      return true if we are a tailsitter transitioning to VTOL flight
    */
    bool in_tailsitter_vtol_transition(uint32_t now = 0) const override;


};
