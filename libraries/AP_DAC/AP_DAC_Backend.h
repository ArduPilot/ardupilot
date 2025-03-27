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
  digitial to analog support, for control of an analog output voltage
 */
#pragma once

#include "AP_DAC_config.h"

#if AP_DAC_ENABLED

class AP_DAC_Backend
{
public:
    AP_DAC_Backend(const AP_DAC_Params &_params) :
        params(_params) {}

    // initialise backend
    virtual void init(void) = 0;

    // update backend
    virtual void update(void) {};

    // set voltage for a channel
    virtual bool set_voltage(uint8_t chan, float v) = 0;

protected:
    const AP_DAC_Params &params;
};

#endif // AP_DAC_ENABLED
