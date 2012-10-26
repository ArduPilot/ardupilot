// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL AVRUARTDriver, based on FastSerial
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;

void setup(void)
{
        //
        // HAL will start serial port at 115200.
        //

        //
        // Test printing things
        //
    hal.uart0->print("test");
    hal.uart0->println(" begin");
    hal.uart0->println(1000);
    hal.uart0->println(1000, 8);
    hal.uart0->println(1000, 10);
    hal.uart0->println(1000, 16);
    hal.uart0->println_P(PSTR("progmem"));
    hal.uart0->printf("printf %d %u %#x %p %f %S\n", -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
    hal.uart0->printf_P(PSTR("printf_P %d %u %#x %p %f %S\n"), -1000, 1000, 1000, 1000, 1.2345, PSTR("progmem"));
    hal.uart0->println("done");
}

void loop(void)
{
    int c;
    //
    // Perform a simple loopback operation.
    //
    c = hal.uart0->read();
    if (-1 != c)
        hal.uart0->write(c);
}


extern "C" {
int main (void) {
    hal.init(NULL);
    setup();
    for(;;) loop();
    return 0;
}
}
