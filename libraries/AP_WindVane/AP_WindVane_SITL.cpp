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

#include "AP_WindVane_SITL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <SITL/SITL.h>
#include <AP_AHRS/AP_AHRS.h>

void AP_WindVane_SITL::update_direction()
{
    if (_frontend._direction_type == _frontend.WindVaneType::WINDVANE_SITL_TRUE) {
        // read in the true wind direction and calculate the apparent

        // temporarily store true speed and direction for easy access
        const float wind_speed = AP::sitl()->wind_speed_active;
        const float wind_dir_rad = radians(AP::sitl()->wind_direction_active);

        // Note than the SITL wind direction is defined as the direction the wind is traveling to
        // This is accounted for in these calculations

        // convert true wind speed and direction into a 2D vector
        Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);

        // add vehicle speed to get apparent wind vector
        wind_vector_ef.x += AP::sitl()->state.speedN;
        wind_vector_ef.y += AP::sitl()->state.speedE;

        _frontend._direction_apparent_raw =  wrap_PI(atan2f(wind_vector_ef.y, wind_vector_ef.x) - AP::ahrs().yaw);

    } else { // WINDVANE_SITL_APARRENT
        // directly read the apparent wind from as set by physics backend
        _frontend._direction_apparent_raw =  wrap_PI(AP::sitl()->get_apparent_wind_dir() - AP::ahrs().yaw);
    }

}

void AP_WindVane_SITL::update_speed()
{
    if (_frontend._speed_sensor_type == _frontend.Speed_type::WINDSPEED_SITL_TRUE) {
        // read in the true wind direction and calculate the apparent

        // temporarily store true speed and direction for easy access
        const float wind_speed = AP::sitl()->wind_speed_active;
        const float wind_dir_rad = radians(AP::sitl()->wind_direction_active);

        // convert true wind speed and direction into a 2D vector
        Vector2f wind_vector_ef(cosf(wind_dir_rad) * wind_speed, sinf(wind_dir_rad) * wind_speed);

        // add vehicle speed to get apparent wind vector
        wind_vector_ef.x += AP::sitl()->state.speedN;
        wind_vector_ef.y += AP::sitl()->state.speedE;

        _frontend._speed_apparent_raw = wind_vector_ef.length();

    } else { // WINDSPEED_SITL_APARRENT
        // directly read the apparent wind from as set by physics backend
        _frontend._speed_apparent_raw = AP::sitl()->get_apparent_wind_spd();
    }
}
#endif
