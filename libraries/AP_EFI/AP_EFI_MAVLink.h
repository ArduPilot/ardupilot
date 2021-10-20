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
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#if HAL_EFI_MAVLINK_ENABLED

class AP_EFI_MAVLink: public AP_EFI_Backend {

public:

    using AP_EFI_Backend::AP_EFI_Backend;

    // Update the state structure
    void update() override;

    void handle_mavlink_msg(const GCS_MAVLINK &channel, const mavlink_message_t &msg) override;

private:

    // we just store the most recent packet for the time being:
    mavlink_efi_status_t packet;
};

#endif  // HAL_EFI_MAVLINK_ENABLED
