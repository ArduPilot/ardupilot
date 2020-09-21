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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_Proximity_TeraRangerTowerEvoGazeboSITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the proximity sensor.
*/
AP_Proximity_TeraRangerTowerEvoGazeboSITL::AP_Proximity_TeraRangerTowerEvoGazeboSITL(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state):
    AP_Proximity_Backend(_frontend, _state),
    _last_timestamp(0),
    sock(true)
{
//  SITL::SITL *sitl = AP::sitl();
  // try to bind to a specific port so that if we restart ArduPilot
  // Gazebo keeps sending us packets. Not strictly necessary but
  // useful for debugging
  sock.bind("127.0.0.1", 9006);

  sock.reuseaddress();
  sock.set_blocking(false);

  hal.console->printf("AP_Proximity_TeraRangerTowerEvoGazeboSITL()\n");
}

// update the state of the sensor
void AP_Proximity_TeraRangerTowerEvoGazeboSITL::update(void)
{
  struct terarangerevoPacket {
    uint64_t timestamp;
    float target[8];
  } pkt;
  const int wait_ms = 0;
  ssize_t s = sock.recv(&pkt, sizeof(terarangerevoPacket), wait_ms);

  if (s == sizeof(terarangerevoPacket) && pkt.timestamp > _last_timestamp) {
    set_status(AP_Proximity::Status::Good);

    memset(_distance_valid, 0, sizeof(_distance_valid));
    memset(_angle, 0, sizeof(_angle));
    memset(_distance, 0, sizeof(_distance));
    _last_timestamp = pkt.timestamp;

    for (uint8_t i = 0; i < 8; i++) {
        _distance_valid[i] = std::isinf(pkt.target[i]) ? false : true;
        _distance[i] = pkt.target[i];
        _angle[i] = i * 45.0f;
//        update_boundary_for_sector(sector, true);
    }

  }

#if 0
    for (uint8_t i=0; i<8; i++) {
        printf("sector[%u] ang=%.1f dist=%.1f\n", i, _angle[i], _distance[i]);
    }
#endif
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_TeraRangerTowerEvoGazeboSITL::distance_max() const
{
    return 60.0f;
}

float AP_Proximity_TeraRangerTowerEvoGazeboSITL::distance_min() const
{
    return 0.5f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_TeraRangerTowerEvoGazeboSITL::get_upward_distance(float &distance) const
{
    // we don't have an upward facing laser
    return false;
}

#endif // CONFIG_HAL_BOARD
