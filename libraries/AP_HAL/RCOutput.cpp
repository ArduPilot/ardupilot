// -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_Progmem/AP_Progmem.h>

#include "AP_HAL.h"
#include "RCOutput.h"

/*
 * Write independent values for each the channel
 *
 * @channel_map: the motor-to-channel map with size specified by @len
 * @enable_map:  the enable map with size specified by @len
 * @enable_map:  the enable map with size specified by @len or NULL if all
 *               channels are enabled
 * @values:      the values to write to each channel
 * @len:         the numer of channels being written
 */
void AP_HAL::RCOutput::write(const uint8_t *channel_map, const bool *enable_map,
                             const int16_t *values, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        if (!enable_map || enable_map[i]) {
            write(pgm_read_byte(&channel_map[i]), values[i]);
        }
    }
}

/*
 * Write the same values for all the channels
 *
 * @channel_map: the motor-to-channel map with size specified by @len
 * @enable_map:  the enable map with size specified by @len or NULL if all
 *               channels are enabled
 * @value:       the value to write to all channels
 * @len:         the numer of channels being written
 */
void AP_HAL::RCOutput::write(const uint8_t *channel_map, const bool *enable_map,
                             int16_t value, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        if (!enable_map || enable_map[i]) {
            write(pgm_read_byte(&channel_map[i]), value);
        }
    }
}
