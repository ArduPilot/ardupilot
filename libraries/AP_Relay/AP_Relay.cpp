// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_HAL.h>
#include "AP_Relay.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define RELAY_PIN 47
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define RELAY_PIN 13
#else
// no relay for this board
#define RELAY_PIN -1
#endif

void AP_Relay::init() {
#if RELAY_PIN != -1
    hal.gpio->pinMode(RELAY_PIN, GPIO_OUTPUT);
#endif
}

void AP_Relay::on() {
#if RELAY_PIN != -1
    hal.gpio->write(RELAY_PIN, 1);
#endif
}


void AP_Relay::off() {
#if RELAY_PIN != -1
    hal.gpio->write(RELAY_PIN, 0);
#endif
}


void AP_Relay::toggle() {
#if RELAY_PIN != -1
    bool ison = hal.gpio->read(RELAY_PIN);
    if (ison)
        off();
    else
        on();
#endif
}

void AP_Relay::set(bool status){
#if RELAY_PIN != -1
    hal.gpio->write(RELAY_PIN, status);
#endif
}

bool AP_Relay::get() {
#if RELAY_PIN != -1
    return hal.gpio->read(RELAY_PIN); 
#else
    return false;
#endif
}
