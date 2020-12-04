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
 * AP_IRLock_SITL.cpp
 *
 *  Created on: June 09, 2016
 *      Author: Ian Chen
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_IRLock_SITL.h"
#include "AP_AHRS/AP_AHRS.h"

void AP_IRLock_SITL::init(int8_t bus)
{
    _sitl = AP::sitl();
    _sitl->precland_sim._type.set_and_notify(SITL::SIM_Precland::PreclandType::PRECLAND_TYPE_CONE);
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_SITL::update()
{
    // return immediately if not healthy
    _flags.healthy = _sitl->precland_sim.healthy();
    if (!_flags.healthy) {
        return false;
    }

    if (_sitl->precland_sim.last_update_ms() != _last_timestamp) {
        const Vector3f position = _sitl->precland_sim.get_target_position();
        const Matrix3f &body_to_ned = AP::ahrs().get_rotation_body_to_ned();
        const Vector3f real_position =  body_to_ned.mul_transpose(-position);
        _last_timestamp = _sitl->precland_sim.last_update_ms();
        _last_update_ms = _last_timestamp;
        _target_info.timestamp = _last_timestamp;
        _target_info.pos_x = real_position.y;
        _target_info.pos_y = -real_position.x;
        _target_info.pos_z = real_position.z;
        return true;
    }
    return false;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
