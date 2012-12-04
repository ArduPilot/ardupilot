
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;

AP_HAL::DigitalSource *a_led;
AP_HAL::DigitalSource *b_led;
AP_HAL::DigitalSource *c_led;

void loop (void) {
    hal.scheduler->delay(1000);
    hal.gpio->write(13, 1);

    a_led->write(1);
    b_led->write(0);
    c_led->write(1);

    hal.scheduler->delay(1000);
    hal.gpio->write(13, 0);

    a_led->write(0);
    b_led->write(1);
    c_led->write(0);
}

void setup (void) {
    hal.gpio->pinMode(13, GPIO_OUTPUT);
    hal.gpio->write(13, 0);

    a_led = hal.gpio->channel(27);
    b_led = hal.gpio->channel(26);
    c_led = hal.gpio->channel(25);

    a_led->mode(GPIO_OUTPUT);
    b_led->mode(GPIO_OUTPUT);
    c_led->mode(GPIO_OUTPUT);

    a_led->write(0);
    b_led->write(0);
    c_led->write(0);
}

AP_HAL_MAIN();
