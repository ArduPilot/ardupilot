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
    if (tailsitter_active()) {
        motors_output();
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
    if (ahrs_view->pitch_sensor < -tailsitter.transition_angle*100 ||
        ahrs_view->roll_sensor < -tailsitter.transition_angle*100) {
        return true;
    }
    // still waiting
    return false;
}
