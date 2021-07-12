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
#include <AP_RangeFinder/AP_RangeFinder.h>

namespace SITL {

class SerialRangeFinder : public SerialDevice {
public:

    SerialRangeFinder(uint8_t port_num): _port_num(port_num) {};

    // update state
    virtual void update(float range, uint8_t health);

    virtual uint32_t packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen) = 0;

    virtual uint16_t reading_interval_ms() const { return 200; } // 5Hz default

    uint8_t get_port() const { return _port_num; }

    virtual void set_health(RangeFinder::Status health) {};

private:

    uint32_t last_sent_ms;
    uint8_t _port_num;
};

}
