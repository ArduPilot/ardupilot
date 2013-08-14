/*
 *  ToshibaLED_PX4 Library. 
 *  DIYDrones 2013
 *
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include "ToshibaLED_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <include/device/rgbled.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

// static private variable instantiation
int ToshibaLED_PX4::_rgbled_fd;         // px4 rgbled driver's file descriptor
bool ToshibaLED_PX4::_enabled;          // true if the led was initialised successfully
bool ToshibaLED_PX4::_healthy;          // true if the led's latest update was successful
uint8_t ToshibaLED_PX4::_red_des;       // desired redness of led
uint8_t ToshibaLED_PX4::_green_des;     // desired greenness of led
uint8_t ToshibaLED_PX4::_blue_des;      // desired blueness of led
uint8_t ToshibaLED_PX4::_red_curr;      // current redness of led
uint8_t ToshibaLED_PX4::_green_curr;    // current greenness of led
uint8_t ToshibaLED_PX4::_blue_curr;     // current blueness of led

void ToshibaLED_PX4::init()
{
    // default LED to healthy
    _healthy = true;

    // open the rgb led device
    _rgbled_fd = open(RGBLED_DEVICE_PATH, 0);
	if (_rgbled_fd < 0) {
        hal.console->printf("Unable to open " RGBLED_DEVICE_PATH);
        _healthy = false;
	}else{
        if (_healthy) {
            _enabled = true;
        }

        // register update with scheduler and AP_Notify's update call
        if (_healthy) {
            hal.scheduler->register_timer_process( ToshibaLED_PX4::_scheduled_update );
            AP_Notify::register_update_function(ToshibaLED_PX4::update);            
            _enabled = true;
        }
    }
}

// set_rgb - set color as a combination of red, green and blue values
void ToshibaLED_PX4::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // send if desired colours aare not already set
    if (green != _green_curr || blue != _blue_curr || red != _red_curr) {
        struct RGBLEDSet v;
        v.red   = red;
        v.green = green;
        v.blue  = blue;
        int ret = ioctl(_rgbled_fd, RGBLED_SET, (unsigned long)&v);
        if (ret >= 0) {
            _green_curr = green;
            _blue_curr = blue;
            _red_curr = red;
            _healthy = true;
        }else{
            _healthy = false;
        }
    }
}

// _scheduled_update - updates _red, _green, _blue according to notify flags
void ToshibaLED_PX4::_scheduled_update(uint32_t now)
{
    static uint8_t counter;     // to reduce 1khz to 10hz
    static uint8_t step;        // holds step of 10hz filter

    // slow rate from 1khz to 10hz
    counter++;
    if (counter < 100) {
        return;
    }

    // reset counter
    counter = 0;

    // move forward one step
    step++;
    if (step>=10) {
        step = 0;
    }

    // initialising pattern
    if (AP_Notify::flags.initialising) {
        if (step & 1) {
            // odd steps display red light
            _red_des = TOSHIBA_LED_DIM;
            _blue_des = TOSHIBA_LED_OFF;
            _green_des = TOSHIBA_LED_OFF;
        }else{
            // even display blue light
            _red_des = TOSHIBA_LED_OFF;
            _blue_des = TOSHIBA_LED_DIM;
            _green_des = TOSHIBA_LED_OFF;
        }

        // exit so no other status modify this pattern
        return;
	}

    // save trim and esc calibration pattern
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration) {
        switch(step) {
            case 0:
            case 3:
            case 6:
                _red_des = TOSHIBA_LED_DIM;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 1:
            case 4:
            case 7:
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_DIM;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 2:
            case 5:
            case 8:
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_DIM;
                break;

            case 9:
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_OFF;
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // solid green or flashing green if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with 3d lock
        if (AP_Notify::flags.gps_status == 3) {
            _red_des = TOSHIBA_LED_OFF;
            _blue_des = TOSHIBA_LED_OFF;
            _green_des = TOSHIBA_LED_DIM;
        }else{
            // flash green if armed with no gps lock
            switch(step) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    _red_des = TOSHIBA_LED_DIM;
                    _blue_des = TOSHIBA_LED_DIM;
                    _green_des = TOSHIBA_LED_DIM;
                    break;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                    _red_des = TOSHIBA_LED_OFF;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_OFF;
                    break;
            }
        }
    }else{
        // flash yellow if failing pre-arm checks
        if (AP_Notify::flags.pre_arm_check == 0) {
            switch(step) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                    _red_des = TOSHIBA_LED_DIM;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_DIM;
                    break;
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                    // even display blue light
                    _red_des = TOSHIBA_LED_OFF;
                    _blue_des = TOSHIBA_LED_OFF;
                    _green_des = TOSHIBA_LED_OFF;
                    break;
            }
        }else{
            // solid blue if gps 3d lock
            if (AP_Notify::flags.gps_status == 3) {
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_DIM;
                _green_des = TOSHIBA_LED_OFF;
            }else{
                // flashing blue if no gps lock
                switch(step) {
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                        _red_des = TOSHIBA_LED_OFF;
                        _blue_des = TOSHIBA_LED_DIM;
                        _green_des = TOSHIBA_LED_OFF;
                        break;
                    case 5:
                    case 6:
                    case 7:
                    case 8:
                    case 9:
                        // even display blue light
                        _red_des = TOSHIBA_LED_OFF;
                        _blue_des = TOSHIBA_LED_OFF;
                        _green_des = TOSHIBA_LED_OFF;
                        break;
                }
            }
        }
    }
}

// update - updates led according to timed_updated.  Should be called regularly from main loop
void ToshibaLED_PX4::update()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    set_rgb(_red_des,_green_des,_blue_des);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4