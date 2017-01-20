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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "NotifyDevice.h"

#define HIGH 1
#define LOW 0

class ExternalLED: public NotifyDevice
{
public:
    // constructor
    ExternalLED() : _counter(0), _counter2(0), _pattern(NONE), _pattern_counter(0) {}

    // initialise the LED driver
    bool init(void);

    // should be called at 50Hz
    void update(void);

private:

    enum LEDPattern {
        NONE = 0,
        FAST_FLASH = 1,
        OSCILLATE = 2
    };

    /// buzzer_flag_type - bitmask of current state and ap_notify states we track
    struct copterleds_flag_type {
        // individual led status
        uint8_t armedled_on         : 1;    // 1 if the armed led is currently on
        uint8_t gpsled_on           : 1;    // 1 if the gps led is currently on
        uint8_t motorled1_on        : 1;    // 1 if motor led #1 is currently on
        uint8_t motorled2_on        : 1;    // 1 if motor led #2 is currently on
    } _flags;

    uint8_t         _counter;           // reduces 50hz calls to 10hz
    uint8_t         _counter2;          // used to control steps of arming and gps leds
    LEDPattern      _pattern;           // current pattern for motor leds
    uint8_t         _pattern_counter;   // used to time on/off of current patter

    // armed_led - set armed light on or off
    void armed_led(bool on_off);

    // gps_led - set gps light on or off
    void gps_led(bool on_off);

    // set_pattern - sets pattern for motor leds
    void set_pattern(LEDPattern pattern_id);

    // motor_led1, motor_led2 - set motor lights on or off
    void motor_led1(bool on_off);
    void motor_led2(bool on_off);
};
