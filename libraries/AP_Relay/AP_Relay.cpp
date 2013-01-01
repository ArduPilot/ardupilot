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
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define RELAY_PIN 26
#else
#error "no RELAY_PIN defined for this board"
#endif

void AP_Relay::init() {
    hal.gpio->pinMode(RELAY_PIN, GPIO_OUTPUT);
}

void AP_Relay::on() {
    hal.gpio->write(RELAY_PIN, 1);
}


void AP_Relay::off() {
    hal.gpio->write(RELAY_PIN, 0);
}


void AP_Relay::toggle() {
    bool ison = hal.gpio->read(RELAY_PIN);
    if (ison)
        off();
    else
        on();
}

void AP_Relay::set(bool status){
    hal.gpio->write(RELAY_PIN, status);
}

bool AP_Relay::get() {
    return hal.gpio->read(RELAY_PIN); 
}
