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

#include "AP_OpticalFlow.h"

#if AP_OPTICALFLOW_ENABLED

extern const AP_HAL::HAL& hal;

OpticalFlow_backend::OpticalFlow_backend(OpticalFlow &_frontend) :
    frontend(_frontend)
{
}

OpticalFlow_backend::~OpticalFlow_backend(void)
{
}

// update the frontend
void OpticalFlow_backend::_update_frontend(const struct OpticalFlow::OpticalFlow_state &state)
{
    frontend.update_state(state);
}

// rotate vector based on user supplied sensor orientation
// resulting vector will be as if sensor was pointing dowards in default orientation
void OpticalFlow_backend::apply_orientation(Vector2f &v) const
{
    // skip rotation in the most common case
    const enum Rotation orient = frontend.get_orientation();
    if (orient == ROTATION_NONE) {
        return;
    }

    // create a 3D vector, rotate and then copy only the xy portions
    Vector3f v3d{v.x, v.y, 0};
    v3d.rotate(orient);
    v.x = v3d.x;
    v.y = v3d.y;
}

#endif
