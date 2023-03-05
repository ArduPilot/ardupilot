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

#include "AP_Radar.h"

#if AP_RADAR_ENABLED

extern const AP_HAL::HAL& hal;

Radar_backend::Radar_backend(AP_Radar &_frontend) :
    frontend(_frontend)
{
}

Radar_backend::~Radar_backend(void)
{
}

// update the frontend
void Radar_backend::_update_frontend(const struct AP_Radar::Radar_state &state)
{
    frontend.update_state(state);
}

#endif
