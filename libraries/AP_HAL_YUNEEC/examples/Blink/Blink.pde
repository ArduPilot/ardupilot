#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::DigitalSource *blue_led;
uint32_t i = 0;

void loop (void) {
	for (i = 0; i < 2000000; i++) /* Wait a bit. */;

    hal.gpio->write(PE8, 1);
    blue_led->write(0);

	for (i = 0; i < 2000000; i++) /* Wait a bit. */;

	hal.gpio->write(PE8, 0);
    blue_led->write(1);

}

void setup (void) {

    hal.gpio->pinMode(PE8, HAL_GPIO_OUTPUT);

    blue_led = hal.gpio->channel(PE9);
    blue_led->mode(HAL_GPIO_OUTPUT);

    hal.gpio->write(PE8, 0);
    blue_led->write(0);

}

AP_HAL_MAIN();
