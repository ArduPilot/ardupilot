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

#include "PixRacerLED.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_V4

extern const AP_HAL::HAL& hal;

PixRacerLED::PixRacerLED() :
    RGBLed(0, 1, 1, 1)
{
}

bool PixRacerLED::hw_init(void)
{
    hal.gpio->write(HAL_GPIO_A_LED_PIN, 0);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, 0);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, 0);
    return true;
}

bool PixRacerLED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    hal.gpio->write(HAL_GPIO_A_LED_PIN, (r > 0));
    hal.gpio->write(HAL_GPIO_B_LED_PIN, (g > 0));
    hal.gpio->write(HAL_GPIO_C_LED_PIN, (b > 0));
    return true;
}

#else
bool PixRacerLED::hw_init(void) { return true; }
bool PixRacerLED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) { return true; }
#endif
