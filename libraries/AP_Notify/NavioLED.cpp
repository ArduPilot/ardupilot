/*
   NavioLED driver
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

#include "NavioLED.h"

#define NAVIO_LED_BRIGHT 0x0    // full brightness
#define NAVIO_LED_MEDIUM 0x7F    // medium brightness
#define NAVIO_LED_DIM    0x4F    // dim brightness
#define NAVIO_LED_OFF    0xFF    // off

NavioLED::NavioLED() : 
    RGBLed(NAVIO_LED_OFF, NAVIO_LED_BRIGHT, NAVIO_LED_MEDIUM, NAVIO_LED_DIM)
{

}
