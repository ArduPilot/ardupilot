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
    virtual bool init(void) override;

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    virtual void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    // update - updates led according to timed_updated.  Should be
    // called at 50Hz
    virtual void update() override;

    // handle LED control, only used when LED_OVERRIDE=1
    virtual void handle_led_control(mavlink_message_t *msg) override;
    
protected:
    // methods implemented in hardware specific classes
    virtual bool hw_init(void) = 0;
    virtual bool hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue) = 0;

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    virtual void _set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    void update_override();
    
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
    void update_colours();
    uint32_t get_colour_sequence() const;
    uint32_t get_colour_sequence_obc() const;

    uint8_t get_brightness(void) const;

#define DEFINE_COLOUR_SEQUENCE(S0, S1, S2, S3, S4, S5, S6, S7, S8, S9)  \
    ((S0) << (0*3) | (S1) << (1*3) | (S2) << (2*3) | (S3) << (3*3) | (S4) << (4*3) | (S5) << (5*3) | (S6) << (6*3) | (S7) << (7*3) | (S8) << (8*3) | (S9) << (9*3))

#define DEFINE_COLOUR_SEQUENCE_SLOW(colour)                       \
    DEFINE_COLOUR_SEQUENCE(colour,colour,colour,colour,colour,OFF,OFF,OFF,OFF,OFF)
#define DEFINE_COLOUR_SEQUENCE_FAILSAFE(colour) \
    DEFINE_COLOUR_SEQUENCE(YELLOW,YELLOW,YELLOW,YELLOW,YELLOW,colour,colour,colour,colour,colour)
#define DEFINE_COLOUR_SEQUENCE_SOLID(colour) \
    DEFINE_COLOUR_SEQUENCE(colour,colour,colour,colour,colour,colour,colour,colour,colour,colour)
#define DEFINE_COLOUR_SEQUENCE_ALTERNATE(colour1, colour2)                      \
    DEFINE_COLOUR_SEQUENCE(colour1,colour2,colour1,colour2,colour1,colour2,colour1,colour2,colour1,colour2)

#define OFF    0
#define BLUE   1
#define GREEN  2
#define RED    4
#define YELLOW (RED|GREEN)
#define WHITE (RED|GREEN|BLUE)

    const uint32_t sequence_initialising = DEFINE_COLOUR_SEQUENCE_ALTERNATE(RED,BLUE);
    const uint32_t sequence_trim_or_esc = DEFINE_COLOUR_SEQUENCE(RED,BLUE,GREEN,RED,BLUE,GREEN,RED,BLUE,GREEN,OFF);
    const uint32_t sequence_failsafe_leak = DEFINE_COLOUR_SEQUENCE_FAILSAFE(WHITE);
    const uint32_t sequence_failsafe_ekf = DEFINE_COLOUR_SEQUENCE_FAILSAFE(RED);
    const uint32_t sequence_failsafe_gps_glitching = DEFINE_COLOUR_SEQUENCE_FAILSAFE(BLUE);
    const uint32_t sequence_failsafe_radio_or_battery = DEFINE_COLOUR_SEQUENCE_FAILSAFE(OFF);

    const uint32_t sequence_armed = DEFINE_COLOUR_SEQUENCE_SOLID(GREEN);
    const uint32_t sequence_armed_nogps = DEFINE_COLOUR_SEQUENCE_SOLID(BLUE);
    const uint32_t sequence_prearm_failing = DEFINE_COLOUR_SEQUENCE(YELLOW,YELLOW,OFF,OFF,YELLOW,YELLOW,OFF,OFF,OFF,OFF);
    const uint32_t sequence_disarmed_good_dgps = DEFINE_COLOUR_SEQUENCE_ALTERNATE(GREEN,OFF);
    const uint32_t sequence_disarmed_good_gps = DEFINE_COLOUR_SEQUENCE_SLOW(GREEN);
    const uint32_t sequence_disarmed_bad_gps = DEFINE_COLOUR_SEQUENCE_SLOW(BLUE);

    uint8_t last_step;
    enum rgb_source_t {
        standard = 0,
        mavlink = 1,
        obc = 2,
    };
    rgb_source_t rgb_source() const;

};
