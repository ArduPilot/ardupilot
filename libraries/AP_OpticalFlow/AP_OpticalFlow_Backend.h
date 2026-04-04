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
#pragma once

/*
  AP_OpticalFlow backend class for ArduPilot
 */

#include "AP_OpticalFlow.h"

#include <AP_HAL/Semaphores.h>

class OpticalFlow_backend
{
    friend class AP_OpticalFlow;

public:
    // constructor
    OpticalFlow_backend(AP_OpticalFlow &_frontend);
    virtual ~OpticalFlow_backend(void);

    CLASS_NO_COPY(OpticalFlow_backend);

    // init - initialise sensor
    virtual void init() {}

    // read latest values from sensor and fill in x,y and totals.
    virtual void update() = 0;

    // handle optical flow mavlink messages
    virtual void handle_msg(const mavlink_message_t &msg) {}

#if HAL_MSP_OPTICALFLOW_ENABLED
    // handle optical flow msp messages
    virtual void handle_msp(const MSP::msp_opflow_data_message_t &pkt) {}
#endif

protected:
    // access to frontend
    AP_OpticalFlow &frontend;

    // update the frontend
    void _update_frontend(const struct AP_OpticalFlow::OpticalFlow_state &state);

    // get the flow scaling parameters
    Vector2f _flowScaler(void) const { return Vector2f(frontend._flowScalerX, frontend._flowScalerY); }

    // get the yaw angle in radians
    float _yawAngleRad(void) const { return cd_to_rad(float(frontend._yawAngle_cd)); }

    // apply yaw angle to a vector
    void _applyYaw(Vector2f &v);

    // get ADDR parameter value
    uint8_t get_address(void) const { return frontend._address; }

    // options parameter values
    enum class Option : uint16_t {
        Stabilised = (1 << 0U)      // sensor is stabilised (e.g. mounted on a gimbal)
    };

    // returns true if an option is enabled
    bool option_is_enabled(Option option) const {
        return ((uint8_t)frontend._options.get() & (uint16_t)option) != 0;
    }

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};
