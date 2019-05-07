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
    bool enabled() const { return enable != 0;}

    // constructor
    Sailboat();

    // setup
    void init();

    // update mainsail's desired angle based on wind speed and direction and desired speed (in m/s)
    void  update_mainsail(float desired_speed);

    // Velocity Made Good, this is the speed we are traveling towards the desired destination
    float get_VMG() const;

    // handle user initiated tack while in acro mode
    void  handle_tack_request_acro();

    // return target heading in radians when tacking (only used in acro)
    float get_tack_heading_rad() const;

    // handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
    void  handle_tack_request_auto();

    // clear tacking state variables
    void  clear_tack();

    // returns true if boat is currently tacking
    bool  tacking() const;

    // returns true if sailboat should take a indirect navigation route to go upwind
    bool  use_indirect_route(float desired_heading_cd) const;

    // calculate the heading to sail on if we cant go upwind
    float calc_heading(float desired_heading_cd);

private:

    // parameters
    AP_Int8 enable;
    AP_Float sail_angle_min;
    AP_Float sail_angle_max;
    AP_Float sail_angle_ideal;
    AP_Float sail_heel_angle_max;
    AP_Float sail_no_go;

    enum Sailboat_Tack {
        TACK_PORT,
        TACK_STARBOARD
    };

    bool currently_tacking;         // true when sailboat is in the process of tacking to a new heading
    float tack_heading_rad;         // target heading in radians while tacking in either acro or autonomous modes
    uint32_t auto_tack_request_ms;  // system time user requested tack in autonomous modes
    uint32_t auto_tack_start_ms;    // system time when tack was started in autonomous mode
};
