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

#include "AP_VisualOdom_Backend.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_VisualOdom_Backend::AP_VisualOdom_Backend(AP_VisualOdom &frontend) :
    _frontend(frontend)
{
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_VisualOdom_Backend::healthy() const
{
    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _last_update_ms) < AP_VISUALODOM_TIMEOUT_MS);
}
