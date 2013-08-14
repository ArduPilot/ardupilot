/*
 *  ToshibaLED Library. 
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
#include "ToshibaLED.h"

extern const AP_HAL::HAL& hal;

// static private variable instantiation
bool ToshibaLED::_enabled;          // true if the led was initialised successfully
bool ToshibaLED::_healthy;          // true if the led's latest update was successful
uint8_t ToshibaLED::_red_des;       // desired redness of led
uint8_t ToshibaLED::_green_des;     // desired greenness of led
uint8_t ToshibaLED::_blue_des;      // desired blueness of led
uint8_t ToshibaLED::_red_curr;      // current redness of led
uint8_t ToshibaLED::_green_curr;    // current greenness of led
uint8_t ToshibaLED::_blue_curr;     // current blueness of led

void ToshibaLED::init()
{
    // default LED to healthy
    _healthy = true;

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _healthy = false;
        return;
    }

    // enable the led
    _healthy = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x03) == 0);

    // update the red, green and blue values to zero
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, TOSHIBA_LED_OFF) == 0);
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, TOSHIBA_LED_OFF) == 0);
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, TOSHIBA_LED_OFF) == 0);

    // register update with scheduler
    if (_healthy) {
        hal.scheduler->register_timer_process( ToshibaLED::_scheduled_update );
        AP_Notify::register_update_function(ToshibaLED::update);
        _enabled = true;
    }

    // give back i2c semaphore
    i2c_sem->give();
}

// on - turn LED on
void ToshibaLED::on()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        //_healthy = false;
        return;
    }

    // try writing to the register
    _healthy = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x03) == 0);

    // give back i2c semaphore
    i2c_sem->give();
}

// off - turn LED off
void ToshibaLED::off()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        //_healthy = false;
        return;
    }

    // try writing to the register
    _healthy = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x00) == 0);

    // give back i2c semaphore
    i2c_sem->give();
}

// set_rgb - set color as a combination of red, green and blue values
void ToshibaLED::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    bool success = true;

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        _healthy = false;
        return;
    }

    // update the red value
    if (red != _red_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, red>>4) == 0) {
            _red_curr = red;
        }else{
            success = false;
        }
    }

    // update the green value
    if (green != _green_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, green>>4) == 0) {
            _green_curr = green;
        }else{
            success = false;
        }
    }

    // update the blue value
    if (blue != _blue_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, blue>>4) == 0) {
            _blue_curr = blue;
        }else{
            success = false;
        }
    }

    // set healthy status
    _healthy = success;

    // give back i2c semaphore
    i2c_sem->give();
}

// _scheduled_update - updates _red, _green, _blue according to notify flags
void ToshibaLED::_scheduled_update(uint32_t now)
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
                    _red_des = TOSHIBA_LED_OFF;
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
        }
        return;
    }else{
        // flash yellow if failing pre-arm checks
        if (!AP_Notify::flags.pre_arm_check) {
            // flashing blue if no gps lock
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
void ToshibaLED::update()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    set_rgb(_red_des,_green_des,_blue_des);
}