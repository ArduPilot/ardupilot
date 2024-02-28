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

#include <AP_HAL/HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#ifndef HAL_GPIO_A_LED_PIN
#define HAL_GPIO_A_LED_PIN        -1
#endif
#ifndef HAL_GPIO_B_LED_PIN
#define HAL_GPIO_B_LED_PIN        -1
#endif
#ifndef HAL_GPIO_C_LED_PIN
#define HAL_GPIO_C_LED_PIN        -1
#endif

extern const AP_HAL::HAL& hal;

PixRacerLED::PixRacerLED() :
    RGBLed(0, 1, 1, 1)
{
}

bool PixRacerLED::init(void)
{
    // when HAL_GPIO_LED_ON is 0 then we must not use pinMode()
    // as it could remove the OPENDRAIN attribute on the pin
#if HAL_GPIO_LED_ON != 0
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);
#endif
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
    return true;
}

bool PixRacerLED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    hal.gpio->write(HAL_GPIO_A_LED_PIN, (r > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, (g > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, (b > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    return true;
}

#else
bool PixRacerLED::init(void) { return true; }
bool PixRacerLED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b) { return true; }
#endif
