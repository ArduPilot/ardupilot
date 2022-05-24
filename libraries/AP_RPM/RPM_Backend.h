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

#include "AP_RPM.h"

class AP_RPM_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_RPM_Backend(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state);

    // we declare a virtual destructor so that RPM drivers can
    // override with a custom destructor if need be
    virtual ~AP_RPM_Backend(void) {}

    // update the state structure. All backends must implement this.
    virtual void update() = 0;

    int8_t get_pin(void) const {
        if (state.instance >= RPM_MAX_INSTANCES) {
            return -1;
        }
        return ap_rpm._params[state.instance].pin.get();
    }

protected:

    AP_RPM &ap_rpm;
    AP_RPM::RPM_State &state;
};
