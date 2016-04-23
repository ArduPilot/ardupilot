
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

const AP_HAL::HAL& hal = AP_HAL_VRBRAIN;

AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;

void loop (void) {
    hal.scheduler->delay(1000);
    //hal.gpio->write(19, 1);

    a_led->write(1);
    b_led->write(0);
    c_led->write(1);

    hal.scheduler->delay(1000);
    //hal.gpio->write(19, 0);

    a_led->write(0);
    b_led->write(1);
    c_led->write(0);
}

void setup (void) {
    //hal.gpio->pinMode(19, OUTPUT);
    //hal.gpio->write(19, 0);

    a_led = hal.gpio->channel(19);
    b_led = hal.gpio->channel(20);
    c_led = hal.gpio->channel(21);

    a_led->mode(OUTPUT);
    b_led->mode(OUTPUT);
    c_led->mode(OUTPUT);

    a_led->write(0);
    b_led->write(0);
    c_led->write(0);
}

AP_HAL_MAIN();
