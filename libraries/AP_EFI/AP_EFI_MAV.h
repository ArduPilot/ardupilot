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

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

/*
 * This class is completely separate from the rest of this library.
 * This class is aimed at decoding incoming MavLink messages,
 * storing them and making them available for LUA script.
 */

class AP_EFI_MAV {
public:
    AP_EFI_MAV();

    static AP_EFI_MAV *get_singleton(void) {
        return singleton;
    }

    void handle_EFI_message(const mavlink_message_t &msg);

    float get_EFI_state(uint8_t index);

private:
    static AP_EFI_MAV *singleton;

    //Stores incoming EFI data
    __mavlink_efi_status_t efi_data;
};

namespace AP {
    AP_EFI_MAV &EFI_MAV();
};
