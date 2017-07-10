/*
//   mRoLED driver
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

#include "mRoLED.h"

#define mRo_LED_BRIGHT  0xFF    // full brightness. Note that upper 5 bits are used
#define mRo_LED_MEDIUM  0x80    // medium brightness
#define mRo_LED_DIM     0x40    // dim
#define mRo_LED_OFF     0x00    // off

mRoLED::mRoLED():
    RGBLed(mRo_LED_OFF, mRo_LED_BRIGHT, mRo_LED_MEDIUM, mRo_LED_DIM)
{

}
