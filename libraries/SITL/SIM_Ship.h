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
  simulate ship takeoff and landing
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

namespace SITL {

/*
  a class for individual simulated vehicles
*/

class ShipSim;
class Ship {
    friend class ShipSim;

private:
    void update(float delta_t);
    
    Vector2f position;
    float heading_deg;
    float yaw_rate;
    float speed;
    ShipSim *sim;
};
        
class ShipSim {
public:
    friend class Ship;
    ShipSim();
    void update(void);

    static const struct AP_Param::GroupInfo var_info[];

    /*
      get a ground speed adjustment for a landed vehicle based on
      whether it is on a ship
     */
    Vector2f get_ground_speed_adjustment(const Location &loc, float &yaw_rate);

private:

    AP_Int8 enable;
    AP_Float speed;
    AP_Float path_size;
    AP_Float deck_size;
    AP_Int8 sys_id;

    Location home;
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;

    bool initialised;
    Ship ship;
    uint32_t last_update_us;

    // reporting period in ms
    const float reporting_period_ms = 200;
    uint32_t last_report_ms;
    uint32_t last_heartbeat_ms;

    SocketAPM mav_socket { false };
    bool mavlink_connected;

    void send_report(void);
};

}  // namespace SITL
