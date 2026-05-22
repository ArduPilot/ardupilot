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

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IBUS2_ENABLED

#include "AP_RCProtocol_Backend.h"

/*
  AP_RCProtocol_IBUS2: RC input backend that reads channel data from
  AP_IBUS2_Slave via its singleton.  The slave owns the UART and does
  all Frame 1/2/3 parsing; this backend just fetches the decoded
  channel values and feeds them to the AP_RCProtocol frontend.
*/
class AP_RCProtocol_IBUS2 : public AP_RCProtocol_Backend {
public:
    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    // Called every RC cycle; pulls channel data from AP_IBUS2_Slave.
    void update() override;

private:
    // Timestamp of the last Frame 1 we already reported, to suppress duplicates.
    uint32_t _last_update_ms;
};

#endif  // AP_RCPROTOCOL_IBUS2_ENABLED
