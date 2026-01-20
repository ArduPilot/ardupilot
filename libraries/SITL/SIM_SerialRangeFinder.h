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

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>

#include "SIM_SerialDevice.h"

namespace SITL {

class SerialRangeFinder : public SerialDevice {
public:

    SerialRangeFinder() {};

    // update state
    virtual void update(float range);

    virtual uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) = 0;

    virtual uint16_t reading_interval_ms() const { return 200; } // 5Hz default

    // Rangefinders that return temperature most likely a depthfinder for boats
    virtual bool has_temperature() const { return false; }
    virtual uint32_t packet_for_temperature(float temperature, uint8_t *buffer, uint8_t buflen) { return 0; }; // 0 length packet by default

private:
    void send_temperature();

    uint32_t last_sent_ms;
};

}
