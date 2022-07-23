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
/*
  Scripting MAVLink class, for easy scripting MAVLink packet support
 */
 
#pragma once

#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BATTERY)
#include <AP_BattMonitor/AP_BattMonitor.h>
#endif

#if HAL_HIGH_LATENCY2_ENABLED
class ScriptingMAVLink {
public:
    ScriptingMAVLink(bool use_mavlink1);

    uint16_t create_high_latency2(uint8_t* msgbuf);

    bool is_high_latency_enabled();

    void set_high_latency_enabled(bool hl_enabled);

    void handle_rx_byte(uint8_t rx_byte);

    // singleton support
    ScriptingMAVLink *get_singleton() {
        return _singleton;
    }
private:
    bool useMAVLink1 = false;
    ScriptingMAVLink* _singleton = nullptr;

    // hold relevant MAVLink data for encoding/decoding messages
    struct {
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink {};

};
#endif