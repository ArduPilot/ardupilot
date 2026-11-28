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
  Plane's part of AP_Vehicle's rate thread, see libraries/AP_Vehicle/rate_thread.cpp.
  The thread runs QuadPlane's VTOL rate controller and sends every output.
 */
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED && AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

/*
  the thread runs QuadPlane's rate controller, so it is off until QuadPlane
  is set up. Tailsitters and tiltrotors write their motors from the main
  loop, which the thread would race, so it stays off for them.
 */
AP_Vehicle::FastRateType Plane::get_rate_thread_type() const
{
    if (!quadplane.available() || quadplane.tailsitter.enabled() || quadplane.tiltrotor.enabled()) {
        return FastRateType::FAST_RATE_DISABLED;
    }
    return FastRateType(quadplane.fast_rate_enable.get());
}

// Q_FSTRATE_DIV, taken as 1 if it is below
uint8_t Plane::get_rate_thread_decimation() const
{
    return uint8_t(MAX(quadplane.fast_rate_decimation.get(), 1));
}

// the thread sends the outputs, see QuadPlane::rate_thread_output()
void Plane::rate_thread_motors_output(bool full_push)
{
    quadplane.rate_thread_output(full_push);
}

#endif  // HAL_QUADPLANE_ENABLED && AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
