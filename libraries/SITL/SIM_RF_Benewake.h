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
  Base class for simulator for the Benewake Serial RangeFinders
*/

#pragma once

#include "SIM_SerialRangeFinder.h"

namespace SITL {

class RF_Benewake : public SerialRangeFinder {
public:

    uint32_t packet_for_alt(float alt_m, uint8_t *buffer, uint8_t buflen) override;

private:

    virtual uint8_t byte4() const = 0;
    virtual uint8_t byte5() const = 0;
    virtual uint8_t byte6() const = 0;
    virtual uint8_t byte7() const = 0;

};

}
