/*
 *  AP_HAL_AVR Notify Library. 
 *
 *  Copyright (c) 2013 David "Buzz" Bussenschutt.  All right reserved.
  *  Rev 1.0 - 1st March 2013
 *
  *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 */
#ifndef __TOSHIBA_LED_PX4_H__
#define __TOSHIBA_LED_PX4_H__

#include <AP_HAL.h>

#include <AP_Notify.h>

class ToshibaLED_PX4 {
public:

    // constructor
    ToshibaLED_PX4() {};

    // init - initialised the LED
    static void init(void);

    // enabled - returns true if the LED was initialised successfully
    static bool enabled() { return _enabled; }

    // healthy - returns true if the LED is operating properly
    static bool healthy() { return _healthy; }

    // on - turn LED on
    static void on() {};

    // off - turn LED off
    static void off() {};

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    static void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    // update - updates led according to timed_updated.  Should be called regularly from main loop
    static void update();

private:
    // private methods
    static void _scheduled_update(uint32_t now);

    // private member variables
    static int _rgbled_fd;                              // px4 rgbled driver's file descriptor
    static bool _enabled;                               // true if the LED is operating properly
    static bool _healthy;                               // true if the LED is operating properly
    static uint8_t _red_des, _green_des, _blue_des;     // color requested by timed update
    static uint8_t _red_curr, _green_curr, _blue_curr;  // current colours displayed by the led
    static uint16_t _counter;                           // used to slow the update rate
};

#endif // __TOSHIBA_LED_PX4_H__
