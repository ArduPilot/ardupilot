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

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_ADSB_ENABLED
#define HAL_SIM_ADSB_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if HAL_SIM_ADSB_ENABLED

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
    ADSB_EMITTER_TYPE type;
    uint64_t stationary_object_created_ms; // allows expiring of slow/stationary objects
};
        
    class ADSB : public SerialDevice {
public:
    ADSB() {};
    void update(const class Aircraft &aircraft);

private:
    uint8_t num_vehicles;
    static const uint8_t num_vehicles_MAX = 200;
    ADSB_Vehicle vehicles[num_vehicles_MAX];
    
    // reporting period in ms
    const float reporting_period_ms = 1000;
    uint32_t last_report_us;
    uint32_t last_update_us;
    uint32_t last_tx_report_ms;
    
    uint32_t last_heartbeat_ms;
    bool seen_heartbeat = false;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;

    struct {
        // socket to telem2 on aircraft
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink {};

    void send_report(const SITL::Aircraft&);
};

}  // namespace SITL

#endif  // HAL_SIM_ADSB_ENABLED
