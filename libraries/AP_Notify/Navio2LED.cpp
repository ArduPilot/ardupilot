/*
   Navio2LED driver
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

#include "Navio2LED.h"

#define NAVIO_LED_BRIGHT 1    // full brightness
#define NAVIO_LED_MEDIUM 1    // medium brightness
#define NAVIO_LED_DIM    1    // dim brightness
#define NAVIO_LED_OFF    0    // off

# define GREEN_PIN        27
# define BLUE_PIN        6
# define RED_PIN        4

extern const AP_HAL::HAL& hal;

Navio2LED::Navio2LED()
    : RGBLed(NAVIO_LED_OFF, NAVIO_LED_BRIGHT, NAVIO_LED_MEDIUM, NAVIO_LED_DIM)
{

}

#include <unistd.h>
bool Navio2LED::hw_init(void)
{
    red_pin = hal.gpio->channel(RED_PIN);
    green_pin = hal.gpio->channel(GREEN_PIN);
    blue_pin = hal.gpio->channel(BLUE_PIN);

    red_pin->mode(HAL_GPIO_OUTPUT);
    green_pin->mode(HAL_GPIO_OUTPUT);
    blue_pin->mode(HAL_GPIO_OUTPUT);
    
    red_pin->write(0);
    green_pin->write(0);
    blue_pin->write(0);
    
    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool Navio2LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    /* We fix the GPIO polarity right here */

    red_pin->write(!red);
    green_pin->write(!green);
    blue_pin->write(!blue);

    return true;
}
