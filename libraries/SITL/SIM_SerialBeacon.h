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
  Base class for serially-attached simulated positioning beacons
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_NOOPLOOP_ENABLED  // currently the only serial beacon

#include "SIM_SerialDevice.h"

#include <AP_Math/AP_Math.h>

namespace SITL {

class SerialBeacon : public SerialDevice {
public:

    SerialBeacon() {}

    // update; works out the vehicle's position relative to the beacon
    // origin and sends beacon data to the autopilot:
    void update();

protected:

    // send beacon data describing the supplied vehicle position (in NED
    // metres relative to the beacon origin); implemented per beacon type:
    virtual void send_data(const Vector3f &pos_ned) = 0;

    virtual uint16_t reading_interval_ms() const { return 200; } // 5Hz default

private:

    uint32_t last_sent_ms;
};

}

#endif  // AP_SIM_NOOPLOOP_ENABLED
