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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"

class RGBLed: public NotifyDevice {
public:
    RGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim);

    // init - initialised the LED
    virtual bool init(void);

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    virtual void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    virtual void update();

    // handle LED control, only used when LED_OVERRIDE=1
    virtual void handle_led_control(mavlink_message_t *msg) override;
    
protected:
    // methods implemented in hardware specific classes
    virtual bool hw_init(void) = 0;
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    virtual void _set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    virtual void update_override();
    
    // meta-data common to all hw devices
    uint8_t _red_des, _green_des, _blue_des;     // color requested by timed update
    uint8_t _red_curr, _green_curr, _blue_curr;  // current colours displayed by the led
    uint8_t _led_off;
    uint8_t _led_bright;
    uint8_t _led_medium;
    uint8_t _led_dim;

    struct {
        uint8_t r, g, b;
        uint8_t rate_hz;
        uint32_t start_ms;
    } _led_override;
    
private:
    virtual void update_colours();
};
