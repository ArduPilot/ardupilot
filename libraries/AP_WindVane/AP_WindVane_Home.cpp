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

#include "AP_WindVane_config.h"

#if AP_WINDVANE_HOME_ENABLED

#include "AP_WindVane_Home.h"

#include <AP_AHRS/AP_AHRS.h>

void AP_WindVane_Home::update_direction()
{
    float direction_apparent_ef = _frontend._home_heading;

    if (_frontend._direction_type == _frontend.WINDVANE_PWM_PIN) {
        RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WIND_VANE_DIR_OFSSET);
        if (chan != nullptr) {
            direction_apparent_ef += chan->norm_input() * radians(45);
        }
    }

    _frontend._direction_apparent_raw = wrap_PI(direction_apparent_ef - AP::ahrs().get_yaw());
}

#endif  // AP_WINDVANE_HOME_ENABLED
