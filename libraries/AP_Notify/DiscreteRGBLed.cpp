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

#include "DiscreteRGBLed.h"

extern const AP_HAL::HAL& hal;

DiscreteRGBLed::DiscreteRGBLed(uint16_t red, uint16_t green, uint16_t blue, bool normal_polarity)
    : RGBLed(normal_polarity ? 0 : 1,
             normal_polarity ? 1 : 0,
             normal_polarity ? 1 : 0,
             normal_polarity ? 1 : 0),
      red_pin_number(red),
      green_pin_number(green),
      blue_pin_number(blue)
{

}

bool DiscreteRGBLed::init(void)
{
    red_pin = hal.gpio->channel(red_pin_number);
    green_pin = hal.gpio->channel(green_pin_number);
    blue_pin = hal.gpio->channel(blue_pin_number);

    red_pin->mode(HAL_GPIO_OUTPUT);
    green_pin->mode(HAL_GPIO_OUTPUT);
    blue_pin->mode(HAL_GPIO_OUTPUT);

    red_pin->write(0);
    green_pin->write(0);
    blue_pin->write(0);

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool DiscreteRGBLed::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    red_pin->write(!!red);
    green_pin->write(!!green);
    blue_pin->write(!!blue);

    return true;
}
