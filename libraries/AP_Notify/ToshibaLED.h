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
#ifndef __TOSHIBA_LED_H__
#define __TOSHIBA_LED_H__

#include <AP_HAL.h>
//#include "AP_HAL_AVR_Namespace.h"
#include <AP_Notify.h>

#define TOSHIBA_LED_ADDRESS 0x55    // default I2C bus address
#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register

#define TOSHIBA_LED_BRIGHT  0x0F    // full brightness
#define TOSHIBA_LED_MEDIUM  0x0A    // medium brightness
#define TOSHIBA_LED_DIM     0x01    // dim      // was 0x05
#define TOSHIBA_LED_OFF     0x00    // dim      // was 0x05

class ToshibaLED {
public:

    // constructor
    ToshibaLED();

    // init - initialised the LED
    static void init(void);

    // healthy - returns true if the LED is operating properly
    static bool healthy() { return _healthy; }

    // on - turn LED on
    static void on();

    // off - turn LED off
    static void off();

    // set_rgb - set color as a combination of red, green and blue levels from 0 ~ 15
    static void set_rgb(uint8_t red, uint8_t green, uint8_t blue);

    // update - updates led according to timed_updated.  Should be called regularly from main loop
    static void update();

private:
    // private methods
    static void _scheduled_update(uint32_t now);

    // private member variables
    static AP_HAL::Semaphore*  _i2c_sem;                // pointer to i2c bus semaphore
    static bool _enabled;                               // true if the LED is operating properly
    static bool _healthy;                               // true if the LED is operating properly
    static uint8_t _red_des, _green_des, _blue_des;     // color requested by timed update
    static uint8_t _red_curr, _green_curr, _blue_curr;  // current colours displayed by the led
    static uint16_t _counter;                           // used to slow the update rate
};

#endif // __TOSHIBA_LED_H__
