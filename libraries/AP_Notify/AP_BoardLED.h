/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

#ifndef __AP_HAL_BOARDLED_H__
#define __AP_HAL_BOARDLED_H__

#include <AP_Common.h>
#include <AP_HAL.h>

#define HIGH 1
#define LOW 0

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
 # define HAL_GPIO_A_LED_PIN        37
 # define HAL_GPIO_B_LED_PIN        36
 # define HAL_GPIO_C_LED_PIN        35
 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
// XXX these are just copied, may not make sense
 # define HAL_GPIO_A_LED_PIN        27
 # define HAL_GPIO_B_LED_PIN        26
 # define HAL_GPIO_C_LED_PIN        25
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#elif CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
 # define HAL_GPIO_A_LED_PIN        13
 # define HAL_GPIO_B_LED_PIN        13
 # define HAL_GPIO_C_LED_PIN        13
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif

class AP_BoardLED
{
public:
    // initialise the LED driver
    void init(void);
    
    // should be called at 50Hz
    void update(void);

private:
    // counter incremented at 50Hz
    uint8_t _counter;
};

#endif // __AP_HAL_BOARDLED_H__
