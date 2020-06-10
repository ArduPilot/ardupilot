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
    void update() override {};

    // send static and dynamic data to ADSB transceiver
    void send_configure(const mavlink_channel_t chan) override;
    void send_dynamic_out(const mavlink_channel_t chan) override;

    void handle_msg(const mavlink_channel_t chan, const mavlink_message_t &msg) override;

private:

    // mavlink handler
    void handle_transceiver_report(mavlink_channel_t chan, const mavlink_message_t &msg);

    void handle_out_cfg(const mavlink_message_t &msg);

    // special helpers for uAvionix workarounds
    uint32_t get_encoded_icao(void);
    uint8_t get_encoded_callsign_null_char();

};

