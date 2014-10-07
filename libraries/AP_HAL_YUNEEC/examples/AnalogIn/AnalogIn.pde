// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL_FLYMAPLE AnalogIn
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Expects pin 15 to be connected to board VCC 3.3V
static AP_HAL::AnalogSource *current_pin;  // PB0
static AP_HAL::AnalogSource *vbat_pin; // PC5

void setup(void)
{
    hal.console->println("AnalogIn test starts");
    vbat_pin = hal.analogin->channel(PC5);
    current_pin = hal.analogin->channel(PB0);
}

void loop(void)
{
    float bat_vol  = vbat_pin->voltage_average();
    float bat_cur = current_pin->voltage_average();
    float vcc = vbat_pin->voltage_average_ratiometric();
    float cur = current_pin->voltage_average_ratiometric();

    hal.console->printf("Pin PC5: %f V\n", vcc);
    hal.console->printf("Pin PB0: %f V\n", cur);
    hal.console->printf("Battary voltage: %f V\n", bat_vol);
    hal.console->printf("Battary current: %f A\n", bat_cur);
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
