#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO

#include "GPIO.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LinuxGPIO_RPI::LinuxGPIO_RPI()
{}

void LinuxGPIO_RPI::init()
{
    // open /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        hal.scheduler->panic("Can't open /dev/mem");
    }

    // mmap GPIO
    gpio_map = mmap(
        NULL,                 // Any adddress in our space will do
        BLOCK_SIZE,           // Map length
        PROT_READ|PROT_WRITE, // Enable reading & writting to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File to map
        GPIO_BASE             // Offset to GPIO peripheral
    );

    close(mem_fd); // No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        hal.scheduler->panic("Can't open /dev/mem");
    }

    gpio = (volatile uint32_t *)gpio_map; // Always use volatile pointer!
}

void LinuxGPIO_RPI::pinMode(uint8_t pin, uint8_t output)
{
    if (output == HAL_GPIO_INPUT) {
        GPIO_MODE_IN(pin);
    } else {
        GPIO_MODE_IN(pin);
        GPIO_MODE_OUT(pin);
    }
}

int8_t LinuxGPIO_RPI::analogPinToDigitalPin(uint8_t pin)
{
    return -1;
}

uint8_t LinuxGPIO_RPI::read(uint8_t pin)
{
    uint32_t value = GPIO_GET(pin);
    return value ? 1: 0;
}

void LinuxGPIO_RPI::write(uint8_t pin, uint8_t value)
{
    if (value == LOW) {
        GPIO_SET_LOW = 1 << pin;
    } else {
        GPIO_SET_HIGH = 1 << pin;
    }
}

void LinuxGPIO_RPI::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* LinuxGPIO_RPI::channel(uint16_t n) {
    return new LinuxDigitalSource(n);
}

/* Interrupt interface: */
bool LinuxGPIO_RPI::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    return true;
}

bool LinuxGPIO_RPI::usb_connected(void)
{
    return false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
