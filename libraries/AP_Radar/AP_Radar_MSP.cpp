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

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Radar_MSP.h"
#include <AP_Common/Location.h>

#if HAL_MSP_RADAR_ENABLED

extern const AP_HAL::HAL& hal;

using namespace MSP;

// detect the device
AP_Radar_MSP *AP_Radar_MSP::detect(AP_Radar &_frontend)
{
    // we assume msp messages will be sent into this driver
    return new AP_Radar_MSP(_frontend);
}

// read latest values from sensor and fill in x,y and totals.
void AP_Radar_MSP::update(void)
{
    // return without updating state if no new data
    if (update_count == 0) {
        return;
    }

    struct AP_Radar::Radar_state state {};

    for (uint8_t i = 0; i < RADAR_MAX_PEERS; i++)
    {
        state.peers[i] = peers[i];
    }

    _update_frontend(state);

    // reset local buffers
    update_count = 0;
}

// handle radar msp messages
void AP_Radar_MSP::handle_msp(const MSP::msp_radar_pos_message_t &pkt)
{
    // record peer state
    uint8_t id = pkt.radar_no;
    peers[id].radar_no = pkt.radar_no;
    peers[id].state = pkt.state;
    peers[id].location = Location(pkt.lat, pkt.lon, pkt.alt, Location::AltFrame::ABSOLUTE);
    peers[id].heading = pkt.heading;
    peers[id].speed = pkt.speed;
    peers[id].lq = pkt.lq;
    peers[id].last_update = AP_HAL::millis();
    update_count++;
}

#endif // HAL_MSP_RADAR_ENABLED
