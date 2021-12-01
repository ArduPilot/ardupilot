/*
  simple test of Random Number Generation
 */

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup(void) {
    hal.console->printf("Running Random Number Generator Test!\n");
}

void loop(void)
{
    uint32_t random_number;
    if (hal.util->get_random_vals((uint8_t*)&random_number, sizeof(random_number))) {
        hal.console->printf("RNG %lx\n", (unsigned long)random_number);
    } else {
        hal.console->printf("RNG failed\n");
    }
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();