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
#ifndef __TOSHIBA_LED_PX4_H__
#define __TOSHIBA_LED_PX4_H__

#include "ToshibaLED.h"
#include "AP_Math.h"
#include "vectorN.h"

class ToshibaLED_PX4 : public ToshibaLED
{
public:
    bool hw_init(void);
    bool hw_set_rgb(uint8_t r, uint8_t g, uint8_t b);
private:
    int _rgbled_fd;
    void update_timer(void);
    
    VectorN<uint8_t,3> last, next;
};

#endif // __TOSHIBA_LED_PX4_H__
