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
  Base class for FrSky telemetery
*/

#include "SIM_Frsky.h"

using namespace SITL;

const char *Frsky::dataid_string(DataID id)
{
    switch (id) {
        case DataID::GPS_ALT_BP: return "GPS_ALT_BP";
        case DataID::TEMP1: return "TEMP1";
        case DataID::FUEL: return "FUEL";
        case DataID::TEMP2: return "TEMP2";
        case DataID::GPS_ALT_AP: return "GPS_ALT_AP";
        case DataID::BARO_ALT_BP: return "BARO_ALT_BP";
        case DataID::GPS_SPEED_BP: return "GPS_SPEED_BP";
        case DataID::GPS_LONG_BP: return "GPS_LONG_BP";
        case DataID::GPS_LAT_BP: return "GPS_LAT_BP";
        case DataID::GPS_COURS_BP: return "GPS_COURS_BP";
        case DataID::GPS_SPEED_AP: return "GPS_SPEED_AP";
        case DataID::GPS_LONG_AP: return "GPS_LONG_AP";
        case DataID::GPS_LAT_AP: return "GPS_LAT_AP";
        case DataID::BARO_ALT_AP: return "BARO_ALT_AP";
        case DataID::GPS_LONG_EW: return "GPS_LONG_EW";
        case DataID::GPS_LAT_NS: return "GPS_LAT_NS";
        case DataID::CURRENT: return "CURRENT";
        case DataID::VFAS: return "VFAS";
    }

    return "UNKNOWN";
}
