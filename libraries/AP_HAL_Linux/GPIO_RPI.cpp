#include <AP_HAL/AP_HAL.h>

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

#define MAX_SIZE_LINE 50

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LinuxGPIO_RPI::LinuxGPIO_RPI()
{}

int LinuxGPIO_RPI::getRaspberryPiVersion() const
{
    char buffer[MAX_SIZE_LINE];
    const char* hardware_description_entry = "Hardware";
    const char* v1 = "BCM2708";
    const char* v2 = "BCM2709";
    char* flag;
    FILE* fd;

    fd = fopen("/proc/cpuinfo", "r");

    while (fgets(buffer, MAX_SIZE_LINE, fd) != NULL) {
        flag = strstr(buffer, hardware_description_entry);

        if (flag != NULL) {
            if (strstr(buffer, v2) != NULL) {
                printf("Raspberry Pi 2 with BCM2709!\n");
                fclose(fd);
                return 2;
            } else if (strstr(buffer, v1) != NULL) {
                printf("Raspberry Pi 1 with BCM2708!\n");
                fclose(fd);
                return 1;
            }
        }
    }

    /* defaults to 1 */
    fprintf(stderr, "Could not detect RPi version, defaulting to 1\n");
    fclose(fd);
    return 1;
}

void LinuxGPIO_RPI::init()
{
    uint32_t address = getRaspberryPiVersion() == 1? GPIO_BASE(BCM2708_PERI_BASE): GPIO_BASE(BCM2709_PERI_BASE);
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
        address    // Offset to GPIO peripheral
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
