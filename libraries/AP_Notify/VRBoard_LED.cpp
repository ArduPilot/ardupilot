/*
   ToshibaLED driver
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
#include "VRBoard_LED.h"

#include <AP_HAL/AP_HAL.h>

#if (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && \
     defined(HAL_GPIO_C_LED_PIN))

#define VRBRAIN_LED_BRIGHT  1    // full brightness
#define VRBRAIN_LED_MEDIUM  1    // medium brightness
#define VRBRAIN_LED_DIM     1    // dim
#define VRBRAIN_LED_OFF     0    // off

extern const AP_HAL::HAL& hal;

VRBoard_LED::VRBoard_LED():
    RGBLed(VRBRAIN_LED_OFF, VRBRAIN_LED_BRIGHT, VRBRAIN_LED_MEDIUM, VRBRAIN_LED_DIM)
{

}
bool VRBoard_LED::hw_init(void){
    // setup the main LEDs as outputs
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);

    // turn all lights off
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
	return true;
}
bool VRBoard_LED::hw_set_rgb(uint8_t r, uint8_t g, uint8_t b){

    if(r > 0){
    	hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
    }
    if (g > 0) {
    	hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    }
    if (b > 0) {
    	hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
    } else {
    	hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    }

	return true;
}

#endif
