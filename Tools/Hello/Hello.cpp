/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup()
{
	hal.console->println("hello world");
}

void loop()
{
	hal.scheduler->delay(1000);
	hal.console->println("*");
}

AP_HAL_MAIN();
