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

class AP_ADSB_uAvionix : public AP_ADSB_Backend {
public:
    // constructor
    AP_ADSB_uAvionix(AP_ADSB &adsb);

    void init() override {}
    void update() override;

    void handle_msg(const mavlink_channel_t chan, const mavlink_message_t &msg) override;

private:

    // send static and dynamic data to ADSB transceiver
    void send_configure();
    void send_dynamic_out();

    // mavlink handler
    void handle_transceiver_report(const mavlink_uavionix_adsb_transceiver_health_report_t &packet);

    void handle_out_cfg(const mavlink_uavionix_adsb_out_cfg_t &packet);

    // special helpers for uAvionix workarounds
    uint32_t get_encoded_icao(void);
    uint8_t get_encoded_callsign_null_char();

    uint32_t            _chan_last_ms;
    mavlink_channel_t   _chan = (mavlink_channel_t)-1;
};

