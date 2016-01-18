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
/// @brief  BEV MOTORS controls the Y6 motors when in VTOL


#ifndef __BEV_MOTORS_H__
#define __BEV_MOTORS_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <RC_Channel.h>
#include <AP_Motors.h>
#include <AP_MotorsY6.h>
#include <AP_Math.h>
//#include <BEV_TransitionState.h>



/// @class      BEV_Motors
class BEV_Motors : public AP_MotorsY6 {
public:
    BEV_Motors(RC_Channel &rc_roll, RC_Channel &rc_pitch, RC_Channel &rc_throttle, RC_Channel &rc_yaw) :
               AP_MotorsY6(rc_roll, rc_pitch, rc_throttle, rc_yaw) {_transition_angle_deg = 0;};
    void setup_motors();
    void set_update_rate(uint16_t speed_hz);
    void set_transition_angle_deg(float transition_angle_deg) {_transition_angle_deg = transition_angle_deg;}
    virtual void output_min();

protected:
    float _transition_angle_deg;
    virtual void output_armed();
    int16_t _motor_out[AP_MOTORS_MAX_NUM_MOTORS];
private:
};
#endif  // BEV_MOTORS

