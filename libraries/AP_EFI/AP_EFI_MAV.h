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

#include "AP_EFI_config.h"

#if AP_EFI_MAV_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_MAV : public AP_EFI_Backend {
public:
	using AP_EFI_Backend::AP_EFI_Backend;

    void update() override;

    void handle_EFI_message(const mavlink_message_t &msg) override;

private:
    bool receivedNewData;
};

#endif  // AP_EFI_MAV_ENABLED
