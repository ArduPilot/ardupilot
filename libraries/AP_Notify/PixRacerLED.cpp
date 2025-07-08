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

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_RGB_ENABLED

#include "PixRacerLED.h"

#include <AP_HAL/HAL.h>

#ifndef AP_NOTIFY_GPIO_LED_RGB_RED_PIN
#error "define AP_NOTIFY_GPIO_LED_RGB_RED_PIN"
#endif
#ifndef AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN
#error "define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN"
#endif
#ifndef AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN
#error "define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN"
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
    hal.gpio->pinMode(AP_NOTIFY_GPIO_LED_RGB_RED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN, HAL_GPIO_OUTPUT);
#endif
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_RED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN, HAL_GPIO_LED_OFF);
    return true;
}

bool PixRacerLED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_RED_PIN, (r > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN, (g > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    hal.gpio->write(AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN, (b > 0)?HAL_GPIO_LED_ON:HAL_GPIO_LED_OFF);
    return true;
}

#endif  // AP_NOTIFY_GPIO_LED_RGB_ENABLED
