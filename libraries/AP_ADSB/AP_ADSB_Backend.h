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

#include "AP_ADSB.h"

#if HAL_ADSB_ENABLED
class AP_ADSB_Backend
{
public:
    // constructor.
    AP_ADSB_Backend(AP_ADSB &frontend, uint8_t instance);

    // we declare a virtual destructor so that ADSB drivers can
    // override with a custom destructor if need be
    virtual ~AP_ADSB_Backend(void) {}

    // static detection function
    static bool detect();

    virtual void update() = 0;

    virtual bool init() { return true; }

protected:

    uint8_t _instance;

    AP_HAL::UARTDriver *_port;

    // references
    AP_ADSB &_frontend;
};
#endif // HAL_ADSB_ENABLED
