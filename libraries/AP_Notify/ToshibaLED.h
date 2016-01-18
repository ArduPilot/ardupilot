/*
 *  AP_Notify Library. 
 * based upon a prototype library by David "Buzz" Bussenschutt.
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

#ifndef __TOSHIBA_LED_H__
#define __TOSHIBA_LED_H__

#include <AP_HAL.h>

#define TOSHIBA_LED_BRIGHT  0xFF    // full brightness
#define TOSHIBA_LED_MEDIUM  0x80    // medium brightness
#define TOSHIBA_LED_DIM     0x11    // dim
#define TOSHIBA_LED_OFF     0x00    // off

class ToshibaLED {
public:

    // init - initialised the LED
    void init(void);

    // healthy - returns true if the LED is operating properly
    bool healthy() { return _healthy; }

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    void update();

protected:
    // methods implemented in hardware specific classes
    virtual bool hw_init(void) = 0;
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

    // meta-data common to all hw devices
    uint8_t counter;
    uint8_t step;
    bool _healthy;                               // true if the LED is operating properly
    uint8_t _red_des, _green_des, _blue_des;     // color requested by timed update
    uint8_t _red_curr, _green_curr, _blue_curr;  // current colours displayed by the led
    void update_colours();
};

#endif // __TOSHIBA_LED_H__
