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
  Base class for serial rangefinders
*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include "SIM_SerialDevice.h"

namespace SITL {

class Frsky : public SerialDevice {
public:

    Frsky() {};

    // update state
    virtual void update() = 0;

protected:

    enum class DataID {
        GPS_ALT_BP          = 0x01,
        TEMP1               = 0x02,
        FUEL                = 0x04,
        TEMP2               = 0x05,
        GPS_ALT_AP          = 0x09,
        BARO_ALT_BP         = 0x10,
        GPS_SPEED_BP        = 0x11,
        GPS_LONG_BP         = 0x12,
        GPS_LAT_BP          = 0x13,
        GPS_COURS_BP        = 0x14,
        GPS_SPEED_AP        = 0x19,
        GPS_LONG_AP         = 0x1A,
        GPS_LAT_AP          = 0x1B,
        BARO_ALT_AP         = 0x21,
        GPS_LONG_EW         = 0x22,
        GPS_LAT_NS          = 0x23,
        CURRENT             = 0x28,
        VFAS                = 0x39,
    };

    const char *dataid_string(DataID id);
};

}
