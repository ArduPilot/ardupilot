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
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
Plane::Plane(void)
    : logger(g.log_bitmask)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

#if PRECISION_LANDING == ENABLED
void Plane::update_precland()
{
  
  //
  // hal.console->printf(" Distance : %f\n", rangefinder.distance_orient(ROTATION_PITCH_270));

  // distance_to_target = 0.0f;
  // target_lander.get_reading(distance_to_target);

    // if (plane.g.rangefinder_landing && rangefinder_state.in_range)
    // {
    //     return precland.update(rangefinder_state.height_estimate,true);
    // }
  

 

    // if((distance_to_target) > 0.0f && (distance_to_target <= 50.0f))
    // {
    //   // hal.console->printf("reading: %f\n", distance_to_target);
    //   return precland.update(distance_to_target * 100.0f, true);  // meters to centi-meters
      
      
    // }

  // alt will be unused if we pass false through as the second parameter:
  return precland.update(0,true);

}
#endif

Plane plane;
AP_Vehicle& vehicle = plane;
