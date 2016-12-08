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
  backend driver class for airspeed
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed.h"

extern const AP_HAL::HAL &hal;

AP_Airspeed_Backend::AP_Airspeed_Backend(AP_Airspeed &_frontend) :
    frontend(_frontend)
{
    sem = hal.util->new_semaphore();
}

AP_Airspeed_Backend::~AP_Airspeed_Backend(void)
{
    delete sem;
}
 

int8_t AP_Airspeed_Backend::get_pin(void) const
{
    return frontend._pin;
}

float AP_Airspeed_Backend::get_psi_range(void) const
{
    return frontend._psi_range;
}

uint8_t AP_Airspeed_Backend::get_bus(void) const
{
    return frontend._bus;
}
