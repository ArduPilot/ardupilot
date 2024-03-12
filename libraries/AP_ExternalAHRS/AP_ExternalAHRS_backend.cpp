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
  parent class for ExternalAHRS backends
 */

#include "AP_ExternalAHRS_backend.h"
#include <AP_AHRS/AP_AHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

AP_ExternalAHRS_backend::AP_ExternalAHRS_backend(AP_ExternalAHRS *_frontend,
                                                 AP_ExternalAHRS::state_t &_state) :
    frontend(*_frontend),
    state(_state)
{}


uint16_t AP_ExternalAHRS_backend::get_rate(void) const
{
    return frontend.get_IMU_rate();
}

bool AP_ExternalAHRS_backend::option_is_set(AP_ExternalAHRS::OPTIONS option) const
{
    return frontend.option_is_set(option);
}

bool AP_ExternalAHRS_backend::in_fly_forward(void) const
{
    return AP::ahrs().get_fly_forward();
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED

