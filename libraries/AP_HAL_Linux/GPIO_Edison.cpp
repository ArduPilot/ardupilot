#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NONE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <mraa.h>

#include "GPIO.h"

using namespace Linux;

static const AP_HAL::HAL &hal = AP_HAL::get_HAL();

GPIO_Edison::GPIO_Edison()
{
}

void GPIO_Edison::init()
{
    mraa_init();
    hal.console->printf("init\n");
}

void GPIO_Edison::pinMode(uint8_t pin, uint8_t output)
{
    hal.console->printf("init\n");
    gpio = mraa_gpio_init(pin);
    if (output == HAL_GPIO_INPUT) {
        mraa_gpio_dir(gpio, MRAA_GPIO_IN);
    } else {
        mraa_gpio_dir(gpio, MRAA_GPIO_OUT);
    }
}

uint8_t GPIO_Edison::read(uint8_t pin)
{
    uint32_t value = mraa_gpio_read(gpio);
    return value ? 1: 0;
}

void GPIO_Edison::write(uint8_t pin, uint8_t value)
{
    mraa_gpio_write(gpio, value);
}

void GPIO_Edison::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_Edison::channel(uint16_t n)
{
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO_Edison::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

#endif
