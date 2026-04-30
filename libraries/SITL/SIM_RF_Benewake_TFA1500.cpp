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
  Simulator for the Benewake TFA1500 serial rangefinder
*/

#include "SIM_config.h"

#if AP_SIM_RF_BENEWAKE_TFA1500_ENABLED

#include "SIM_RF_Benewake_TFA1500.h"

using namespace SITL;

uint32_t RF_Benewake_TFA1500::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t PACKET_SIZE = 5;
    if (buflen < PACKET_SIZE) {
        return 0;
    }

    const uint32_t alt_cm = alt_m * 100;

    buffer[0] = 0x5C;
    buffer[1] = alt_cm;
    buffer[2] = (alt_cm >> 8);
    buffer[3] = (alt_cm >> 16);
    buffer[4] = (uint8_t)~(buffer[1] + buffer[2] + buffer[3]);

    return PACKET_SIZE;
}
#endif // AP_SIM_RF_BENEWAKE_TFA1500_ENABLED
