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
#include <AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Expects pin 15 to be connected to board VCC 3.3V
static AP_HAL::AnalogSource *vcc_pin;  // GPIO pin 15
static AP_HAL::AnalogSource *batt_pin; // GPIO pin 20

void setup(void)
{
    hal.console->println("AnalogIn test starts");
    vcc_pin      = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC);
    batt_pin = hal.analogin->channel(20);
}

void loop(void)
{
    float vcc  = vcc_pin->voltage_average();
    float batt = batt_pin->voltage_average();

// Flymaple board pin 20 is connected to the external battery supply
// via a 24k/5.1k voltage divider. The schematic claims the divider is 25k/5k, 
// but the actual installed resistors are not so.
    batt *= 5.70588; // (24000+5100)/5100
    hal.console->printf("Vcc pin 15:       %f\n", vcc);
    hal.console->printf("VIN (via pin 20): %f\n", batt);
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
