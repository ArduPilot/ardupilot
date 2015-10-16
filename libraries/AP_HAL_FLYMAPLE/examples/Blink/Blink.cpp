
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Progmem/AP_Progmem.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
    hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
    hal.gpio->write(13, 0);

    a_led = hal.gpio->channel(27);
    b_led = hal.gpio->channel(26);
    c_led = hal.gpio->channel(25);

    a_led->mode(HAL_GPIO_OUTPUT);
    b_led->mode(HAL_GPIO_OUTPUT);
    c_led->mode(HAL_GPIO_OUTPUT);

    a_led->write(0);
    b_led->write(0);
    c_led->write(0);
}

AP_HAL_MAIN();
