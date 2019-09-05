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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "WheelEncoder_SITL_Quadrature.h"

extern const AP_HAL::HAL& hal;

AP_WheelEncoder_SITL_Qaudrature::AP_WheelEncoder_SITL_Qaudrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
    AP_WheelEncoder_Backend(frontend, instance, state),
    _sitl(AP::sitl())
{
}

void AP_WheelEncoder_SITL_Qaudrature::update(void)
{
    // copy distance and error count so it is accessible to front end
    uint32_t distance_count = 0; // calculated from an old_distance_count and distance travelled?
}

#endif