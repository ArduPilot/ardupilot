/*
  simple test of RC output interface
 */

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void) 
{
    hal.console->println("Starting AP_HAL::RCOutput test");
    for (uint8_t i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
    }
}

static uint16_t pwm = 1500;
static int8_t delta = 1;

void loop (void) 
{
    uint8_t i;
    for (i=0; i<14; i++) {
        hal.rcout->write(i, pwm);
        pwm += delta;
        if (delta > 0 && pwm >= 2000) {
            delta = -1;
            hal.console->printf("reversing\n");
        } else if (delta < 0 && pwm <= 1000) {
            delta = 1;
            hal.console->printf("reversing\n");
        }
    }
    hal.scheduler->delay(5);
}

AP_HAL_MAIN();
