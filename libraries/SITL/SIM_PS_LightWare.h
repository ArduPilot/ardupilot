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
  Base class for LightWare Proximity Sensor serial devices.

*/

#pragma once

#include "SIM_SerialProximitySensor.h"

#ifndef HAL_SIM_PS_LIGHTWARE_ENABLED
#define HAL_SIM_PS_LIGHTWARE_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#if HAL_SIM_PS_LIGHTWARE_ENABLED

#include <stdio.h>

namespace SITL {

class PS_LightWare : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

private:

};

};

#endif  // HAL_SIM_PS_LIGHTWARE_ENABLED
