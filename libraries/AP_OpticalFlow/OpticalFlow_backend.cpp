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

#include "OpticalFlow.h"

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

// apply yaw angle to a vector
void OpticalFlow_backend::_applyYaw(Vector2f &v)
{
    float yawAngleRad = _yawAngleRad();
    if (is_zero(yawAngleRad)) {
        return;
    }
    float cosYaw = cosf(yawAngleRad);
    float sinYaw = sinf(yawAngleRad);
    float x = v.x;
    float y = v.y;
    v.x = cosYaw * x - sinYaw * y;
    v.y = sinYaw * x + cosYaw * y;
}
