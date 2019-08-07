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
    Rover Sailboat functionality
*/
class Sailboat
{
public:

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // enabled
    bool sail_enabled() const { return enable != 0;}

    // Should we use sailboat navigation?
    // currently this is just for tracking upwind
    bool nav_enabled();

    // constructor
    Sailboat();

    // setup
    void init();

    // initialise rc input (channel_mainsail)
    void init_rc_in();

    // decode pilot mainsail input and return in steer_out and throttle_out arguments
    // mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
    void get_pilot_desired_mainsail(float &mainsail_out);

    // update mainsail's desired angle based on wind speed and direction and desired speed (in m/s)
    // we return the throttle to ouput, this may be zeroed if the sail controller does not request throttle
    float update_sail_control(float desired_speed, float throttle_out);

    // Velocity Made Good, this is the speed we are traveling towards the desired destination
    float get_VMG() const;

    // handle user initiated tack while in acro mode
    void  handle_tack_request_acro();

    // return target heading in radians when tacking (only used in acro)
    float get_tack_heading_rad();

    // handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
    void  handle_tack_request_auto();

    // clear tacking state variables
    void  clear_tack();

    // returns true if boat is currently tacking
    bool  tacking();

    // returns true if sailboat should take a indirect navigation route to go upwind
    bool  use_indirect_route(float desired_heading_cd);

    // calculate the heading to sail on if we cant go upwind
    float calc_heading(float desired_heading_cd);

    // return sailboat loiter radius
    float get_loiter_radius() const {return loit_radius;}

    // check aux throttle at arming
    bool aux_throttle_pre_arm_check();

    enum Sailboat_Throttle {
        NEVER        = 0,
        ASSIST       = 1,
        FORCE_MOTOR  = 2
    };
    enum Sailboat_Throttle throttle_state_t; // throttle state used with throttle enum

private:

    // parameters
    AP_Int8 enable;
    AP_Float sail_angle_min;
    AP_Float sail_angle_max;
    AP_Float sail_angle_ideal;
    AP_Float sail_heel_angle_max;
    AP_Float sail_no_go;
    AP_Float max_cross_track;
    AP_Float loit_radius;
    AP_Float sail_assist_windspeed;

    enum Sailboat_Tack {
        TACK_PORT,
        TACK_STARBOARD
    };
    Sailboat_Tack current_tack;  // the tack the nav controller in calc_heading thinks we are on

    enum options {
        AUX_THROTTLE       = (1 << 0),
        AUX_THROTTLE_LIMIT = (1 << 1)
    };

    RC_Channel *channel_mainsail;   // rc input channel for controlling mainsail
    bool currently_tacking;         // true when sailboat is in the process of tacking to a new heading
    float tack_heading_rad;         // target heading in radians while tacking in either acro or autonomous modes
    uint32_t tack_request_ms;       // system time user requested tack
    uint32_t auto_tack_start_ms;    // system time when tack was started in autonomous mode
    bool tack_assist;               // true if we should use some throttle to assist tack

    // Check if we should assist with throttle
    bool throttle_assist();
};
