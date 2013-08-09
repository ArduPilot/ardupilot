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
uint16_t ToshibaLED::_counter;
bool ToshibaLED::_enabled;          // true if the led was initialised successfully
bool ToshibaLED::_healthy;          // true if the led's latest update was successful
uint8_t ToshibaLED::_red_des;       // desired redness of led
uint8_t ToshibaLED::_green_des;     // desired greenness of led
uint8_t ToshibaLED::_blue_des;      // desired blueness of led
uint8_t ToshibaLED::_red_curr;      // current redness of led
uint8_t ToshibaLED::_green_curr;    // current greenness of led
uint8_t ToshibaLED::_blue_curr;     // current blueness of led

// constructor
ToshibaLED::ToshibaLED()
{
}

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

    // set i2c bus to low speed - it seems to work at high speed even though the datasheet doesn't say this
    //hal.i2c->setHighSpeed(false);

    // enable the led
    _healthy = (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_ENABLE, 0x03) == 0);

    // update the red, green and blue values to zero
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, TOSHIBA_LED_OFF) == 0);
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, TOSHIBA_LED_OFF) == 0);
    _healthy &= (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, TOSHIBA_LED_OFF) == 0);

    // register update with scheduler
    if (_healthy) {
        hal.scheduler->register_timer_process( ToshibaLED::_scheduled_update );	
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

    // update the green value
    if (green != _green_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM0, green) == 0) {
            _green_curr = green;
        }else{
            success = false;
        }
    }

    // update the blue value
    if (blue != _blue_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM1, blue) == 0) {
            _blue_curr = blue;
        }else{
            success = false;
        }
    }

    // update the red value
    if (red != _red_curr) {
        if (hal.i2c->writeRegister(TOSHIBA_LED_ADDRESS, TOSHIBA_LED_PWM2, red) == 0) {
            _red_curr = red;
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
    _counter++;

    // we never want to update LEDs at a higher than 16Hz rate
    if (_counter & 0x3F) {
        return;
    }

    // counter2 used to drop frequency down to 16hz
    uint8_t counter2 = _counter >> 6;

    // initialising pattern
    static uint8_t init_counter = 0;
    init_counter++;
    if (notify.flags.initialising) {
        // blink LED red and blue alternatively
        if (init_counter == 1) {
            // turn  on red
            _red_des = TOSHIBA_LED_DIM;
            _blue_des = 0;
            _green_des = 0;
        }else{
            // turn on blue
            _red_des = 0;
            _blue_des = TOSHIBA_LED_DIM;
            _green_des = 0;
            init_counter = 0;
        }
        return;
	}

    // save trim pattern
    if (notify.flags.save_trim) {
        static uint8_t save_trim_counter = 0;
        if ((counter2 & 0x2) == 0) {
            save_trim_counter++;
        }
        switch(save_trim_counter) {
            case 0:
                _red_des = TOSHIBA_LED_DIM;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 1:
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_DIM;
                _green_des = TOSHIBA_LED_OFF;
                break;

            case 2:
                _red_des = TOSHIBA_LED_OFF;
                _blue_des = TOSHIBA_LED_OFF;
                _green_des = TOSHIBA_LED_DIM;
                break;

            default:
                save_trim_counter = -1;
        }
        return;
    }

    // armed and gps
    static uint8_t arm_or_gps = 0;  // 0 = displaying arming state, 1 = displaying gps state
    // switch between showing arming and gps state every second
    if (counter2 == 0) {
        arm_or_gps = !arm_or_gps;
        // turn off both red and blue
        _red_des = TOSHIBA_LED_OFF;
        _blue_des = TOSHIBA_LED_OFF;
        _green_des = TOSHIBA_LED_OFF;
    }

    // displaying arming state
    if (arm_or_gps == 0) {
        static uint8_t arm_counter = 0;
        if (notify.flags.armed) {
            // red led solid
            _red_des = TOSHIBA_LED_DIM;
        }else{
            if ((counter2 & 0x2) == 0) {
                arm_counter++;
            }
            if (notify.flags.pre_arm_check) {
                // passed pre-arm checks so slower single flash
                switch(arm_counter) {
                    case 0:
                    case 1:
                    case 2:
                        _red_des = TOSHIBA_LED_DIM;
                        break;
                    case 3:
                    case 4:
                    case 5:
                        _red_des = TOSHIBA_LED_OFF;
                        break;
                    default:
                        // reset counter to restart the sequence
                        arm_counter = -1;
                        break;
                }
            }else{
                // failed pre-arm checks so double flash
                switch(arm_counter) {
                    case 0:
                    case 1:
                        _red_des = TOSHIBA_LED_DIM;
                        break;
                    case 2:
                        _red_des = TOSHIBA_LED_OFF;
                        break;
                    case 3:
                    case 4:
                        _red_des = TOSHIBA_LED_DIM;
                        break;
                    case 5:
                    case 6:
                        _red_des = TOSHIBA_LED_OFF;
                        break;
                    default:
                        arm_counter = -1;
                        break;
                }
            }
        }
    }else{
        // gps light
        switch (notify.flags.gps_status) {
            case 0:
                // no GPS attached
                _blue_des = TOSHIBA_LED_OFF;
                break;

            case 1:
                // GPS attached but no lock, blink at 4Hz
                if ((counter2 & 0x3) == 0) {
                    // toggle blue
                    if (_blue_des == TOSHIBA_LED_OFF) {
                        _blue_des = TOSHIBA_LED_DIM;
                    }else{
                        _blue_des = TOSHIBA_LED_OFF;
                    }
                }
                break;

            case 2:
                // GPS attached but 2D lock, blink more slowly (around 2Hz)
                if ((counter2 & 0x7) == 0) {
                    // toggle blue
                    if (_blue_des == TOSHIBA_LED_OFF) {
                        _blue_des = TOSHIBA_LED_DIM;
                    }else{
                        _blue_des = TOSHIBA_LED_OFF;
                    }
                }
                break;

            case 3:
                // 3D lock so become solid
                _blue_des = TOSHIBA_LED_DIM;
                break;        
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

    set_rgb(_red_des,_blue_des,_green_des);
}