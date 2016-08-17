/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  ADSB peripheral simulator class
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a class for individual simulated vehicles
*/
class ADSB_Vehicle {
    friend class ADSB;

private:
    void update(float delta_t);
    
    Vector3f position; // NED from origin
    Vector3f velocity_ef; // NED
    char callsign[9];
    uint32_t ICAO_address;
    bool initialised = false;
};
        
class ADSB {
public:
    ADSB(const struct sitl_fdm &_fdm, const char *home_str);
    void update(void);

private:
    const struct sitl_fdm &fdm;
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;

    Location home;
    uint8_t num_vehicles = 0;
    static const uint8_t num_vehicles_MAX = 200;
    ADSB_Vehicle vehicles[num_vehicles_MAX];
    
    // reporting period in ms
    const float reporting_period_ms = 1000;
    uint32_t last_report_us = 0;
    uint32_t last_update_us = 0;
    uint32_t last_tx_report_ms = 0;
    
    uint32_t last_heartbeat_ms = 0;
    bool seen_heartbeat = false;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;

    SocketAPM mav_socket { false };
    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink {};

    void send_report(void);
};

}  // namespace SITL
