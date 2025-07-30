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
  Base class for serial rangefinders
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SERIALPROXIMITYSENSOR_ENABLED

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include "SIM_SerialDevice.h"

namespace SITL {

class SerialProximitySensor : public SerialDevice {
public:

    SerialProximitySensor() {};

    // update state
    virtual void update(const Location &location);

    virtual uint16_t reading_interval_ms() const { return 200; } // 5Hz default

    virtual uint32_t packet_for_location(const Location &location,
                                         uint8_t *data,
                                         uint8_t buflen) = 0;

    // return distance to nearest object at angle
    float measure_distance_at_angle_bf(const Location &location, float angle) const {
        return AP::sitl()->measure_distance_at_angle_bf(location, angle);
    }

private:

    uint32_t last_sent_ms;

};

}

#endif
