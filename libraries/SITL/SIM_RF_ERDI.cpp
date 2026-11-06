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
  Simulator for ERDI TECH serial rangefinders
*/

#include "SIM_config.h"

#if AP_SIM_RF_ERDI_ENABLED

#include "SIM_RF_ERDI.h"

using namespace SITL;

uint32_t RF_ERDI::packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t PACKET_LEN = 4;
    if (buflen < PACKET_LEN) {
        abort();
    }

    const uint16_t raw = raw_for_alt(alt_m);

    buffer[0] = 0x5C;
    buffer[1] = raw & 0xFF;
    buffer[2] = raw >> 8;
    buffer[3] = ~uint8_t(buffer[1] + buffer[2]);

    return PACKET_LEN;
}

#endif  // AP_SIM_RF_ERDI_ENABLED
