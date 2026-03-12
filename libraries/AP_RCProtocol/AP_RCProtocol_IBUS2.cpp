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
 */

#include "AP_RCProtocol_IBUS2.h"

#if AP_RCPROTOCOL_IBUS2_ENABLED

#include <AP_IBus2/AP_IBus2_Slave.h>
#include <AP_Math/AP_Math.h>

void AP_RCProtocol_IBUS2::update()
{
    const AP_IBus2_Slave *slave = AP_IBus2_Slave::get_singleton();
    if (slave == nullptr) {
        return;
    }

    const uint32_t last_ms = slave->get_rc_last_update_ms();
    if (last_ms == 0 || last_ms == _last_update_ms) {
        return;
    }
    _last_update_ms = last_ms;

    // Use a buffer large enough for the slave's maximum channel count.
    uint16_t channels[32];
    uint8_t count;
    if (!slave->get_rc_channels(channels, count)) {
        return;
    }

    count = MIN(count, (uint8_t)MAX_RCIN_CHANNELS);
    add_input(count, channels, slave->get_failsafe());
}

#endif  // AP_RCPROTOCOL_IBUS2_ENABLED
