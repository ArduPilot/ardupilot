// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-
// This code is placed into the public domain.

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>


const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void)
{
    //
    // Test printing things
    //
    hal.scheduler->delay(5000); // Need time to connect to USB port
    hal.console->print("test");
    hal.console->println(" begin");
    hal.console->println(1000);
    hal.console->println(1000, 8);
    hal.console->println(1000, 10);
    hal.console->println(1000, 16);
    hal.console->println("progmem");
    hal.console->printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, "progmem");
    hal.console->printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, "progmem");

    hal.console->println("done.");
    for(;;);

}


void loop(void){}

AP_HAL_MAIN();
