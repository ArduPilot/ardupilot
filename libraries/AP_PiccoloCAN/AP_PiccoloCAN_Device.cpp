/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_PiccoloCAN_Device.h"


/*
 * Determine if this particular device is "connected"
 * (i.e. have we received a CAN frame from this device within the specified timeout period) 
 */
bool AP_PiccoloCAN_Device::is_connected(uint64_t timeout_ms)
{
    uint64_t now = AP_HAL::micros64();

    // No messages have been received at all
    if (last_rx_msg_timestamp == 0) {
        return false;
    }

    return now < (last_rx_msg_timestamp + (timeout_ms * 1000ULL));
}
