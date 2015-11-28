/*
   BHLED driver
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

*/

#include "BHLED.h"

#define BH_LED_BRIGHT 0x00    // full brightness
#define BH_LED_MEDIUM 0x7F    // medium brightness
#define BH_LED_DIM    0x4F    // dim brightness
#define BH_LED_OFF    0xFF    // off

BHLED::BHLED() :
    RGBLed(BH_LED_OFF, BH_LED_BRIGHT, BH_LED_MEDIUM, BH_LED_DIM)
{

}
