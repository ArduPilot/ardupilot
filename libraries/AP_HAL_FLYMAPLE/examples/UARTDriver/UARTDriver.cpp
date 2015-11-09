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
    hal.uartA->begin(115200);
    
    //
    // Test printing things
    //
        
    hal.uartA->print("test");
    hal.uartA->println(" begin");
    hal.uartA->println(1000);
    hal.uartA->println(1000, 8);
    hal.uartA->println(1000, 10);
    hal.uartA->println(1000, 16);
    hal.uartA->println("progmem");
    int x = 3;
    int *ptr = &x;
    hal.uartA->printf("printf %d %u %#x %p %f %s\n", -1000, 1000, 1000, ptr, 1.2345, "progmem");
    hal.uartA->printf("printf %d %u %#x %p %f %s\n", -1000, 1000, 1000, ptr, 1.2345, "progmem");
    hal.uartA->println("done");
}

void loop(void)
{
    int c;
    //
    // Perform a simple loopback operation.
    //
    c = hal.uartA->read();
    if (-1 != c)
        hal.uartA->write(c);
}

AP_HAL_MAIN();
