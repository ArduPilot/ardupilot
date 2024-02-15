#pragma once

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

#include "AP_ADSB_Backend.h"

#if HAL_ADSB_UAVIONIX_MAVLINK_ENABLED
class AP_ADSB_uAvionix_MAVLink : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    void update() override;

    // static detection function
    static bool detect();

private:
    // send static and dynamic data to ADSB transceiver
    void send_configure(const mavlink_channel_t chan);
    void send_dynamic_out(const mavlink_channel_t chan) const;

    // special helpers for uAvionix workarounds
    uint32_t encode_icao(const uint32_t icao_id) const;
    uint8_t get_encoded_callsign_null_char(void);
};
#endif // HAL_ADSB_ENABLED

