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

# define GPIO_A_LED_PIN        27
# define GPIO_B_LED_PIN        6
# define GPIO_C_LED_PIN        4
# define GPIO_LED_ON           0
# define GPIO_LED_OFF          1

extern const AP_HAL::HAL& hal;

Navio2LED::Navio2LED():
    RGBLed(NAVIO_LED_OFF, NAVIO_LED_BRIGHT, NAVIO_LED_MEDIUM, NAVIO_LED_DIM)
{

}

bool Navio2LED::hw_init(void)
{
    // setup the main LEDs as outputs
    hal.gpio->pinMode(GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);

    // turn all lights off
    hal.gpio->write(GPIO_A_LED_PIN, GPIO_LED_OFF);
    hal.gpio->write(GPIO_B_LED_PIN, GPIO_LED_OFF);
    hal.gpio->write(GPIO_C_LED_PIN, GPIO_LED_OFF);

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool Navio2LED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red == NAVIO_LED_OFF) {
        hal.gpio->write(GPIO_C_LED_PIN, GPIO_LED_OFF);
    } else {
        hal.gpio->write(GPIO_C_LED_PIN, GPIO_LED_ON);
    }
    if (green == NAVIO_LED_OFF) {
        hal.gpio->write(GPIO_A_LED_PIN, GPIO_LED_OFF);
    } else {
        hal.gpio->write(GPIO_A_LED_PIN, GPIO_LED_ON);
    }
    if (blue == NAVIO_LED_OFF) {
        hal.gpio->write(GPIO_B_LED_PIN, GPIO_LED_OFF);
    } else {
        hal.gpio->write(GPIO_B_LED_PIN, GPIO_LED_ON);
    }

    return true;
}
