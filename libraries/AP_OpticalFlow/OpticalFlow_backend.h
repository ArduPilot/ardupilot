#ifndef __OpticalFlow_backend_H__
#define __OpticalFlow_backend_H__
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
  OpticalFlow backend class for ArduPilot
 */

#include "OpticalFlow.h"

class OpticalFlow_backend
{
    friend class OpticalFlow;

public:
    // constructor
    OpticalFlow_backend(OpticalFlow &_frontend) : frontend(_frontend) {}

    // init - initialise sensor
    virtual void init() = 0;

    // read latest values from sensor and fill in x,y and totals.
    virtual void update() = 0;

protected:
    // access to frontend
    OpticalFlow &frontend;

    // update the frontend
    void _update_frontend(const struct OpticalFlow::OpticalFlow_state &state);

    // get the flow scaling parameters
    Vector2f _flowScaler(void) const { return Vector2f(frontend._flowScalerX, frontend._flowScalerY); }

    // get the yaw angle in radians
    float _yawAngleRad(void) const { return radians(float(frontend._yawAngle_cd) * 0.01f); }
};

#endif // __OpticalFlow_backend_H__

