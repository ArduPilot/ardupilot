/*
  ToshibaLED PX4 driver
*/
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
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>

#include "ToshibaLED.h"

class ToshibaLED_PX4 : public ToshibaLED
{
public:
    bool hw_init(void);
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b);
private:
    int _rgbled_fd;
    void update_timer(void);

    // use a union so that updates can be of a single 32 bit value,
    // making it atomic on PX4
    union rgb_value {
        struct {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };
        volatile uint32_t v;
    };
    
    union rgb_value last, next;
};
