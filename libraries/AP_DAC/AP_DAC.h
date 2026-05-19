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
  digital to analog support, for control of an analog output voltage
 */
#pragma once

#include "AP_DAC_config.h"

#if AP_DAC_ENABLED

#include "AP_DAC_Params.h"
#include "AP_DAC_Backend.h"

class AP_DAC
{
public:
    // Constructor
    AP_DAC();

    CLASS_NO_COPY(AP_DAC);

    // detect and initialise any available DACs
    void init();

    // update all of the backends
    void update();

    // set voltage for a channel
    bool set_voltage(uint8_t instance, uint8_t chan, float v);

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_DAC_Params params[AP_DAC_MAX_INSTANCES];
    AP_DAC_Backend *backends[AP_DAC_MAX_INSTANCES];
};

#endif // AP_DAC_ENABLED
