#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET

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

#define LOW             0
#define HIGH            1

using namespace Linux;

GPIO_BBB::GPIO_BBB()
{}

void GPIO_BBB::init()
{
#if LINUX_GPIO_NUM_BANKS == 4
    int mem_fd;

    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

    /*
    Enable all GPIO clocks
    Without this, access to deactivated banks (i.e. those with no clock source set up) will (logically) fail with SIGBUS
     */
    volatile unsigned *cm_per = (volatile unsigned *)mmap(0, CM_PER_BASE_SIZE, PROT_READ|PROT_WRITE,
                                MAP_SHARED, mem_fd, CM_PER_BASE);
    if ((char *)cm_per == MAP_FAILED) {
        AP_HAL::panic("unable to map CM_PER registers");
    }
    off_t cm_offsets[LINUX_GPIO_NUM_BANKS-1] = { CM_PER_GPIO1_CLKCTRL, CM_PER_GPIO2_CLKCTRL, CM_PER_GPIO3_CLKCTRL };
    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS-1; i++) {
        unsigned reg_value = *(cm_per + cm_offsets[i]);
        *(cm_per + cm_offsets[i]) = (reg_value & ~0b11) | 0b10;
    }
    munmap((void *)cm_per, CM_PER_BASE_SIZE);

    /* mmap GPIO */
    off_t offsets[LINUX_GPIO_NUM_BANKS] = { GPIO0_BASE, GPIO1_BASE, GPIO2_BASE, GPIO3_BASE };
    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS; i++) {
        gpio_bank[i].base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, offsets[i]);
        if ((char *)gpio_bank[i].base == MAP_FAILED) {
            AP_HAL::panic("unable to map GPIO bank");
        }
        gpio_bank[i].oe = gpio_bank[i].base + GPIO_OE;
        gpio_bank[i].in = gpio_bank[i].base + GPIO_IN;
        gpio_bank[i].out = gpio_bank[i].base + GPIO_OUT;
    }

    close(mem_fd);
#endif // LINUX_GPIO_NUM_BANKS
}

void GPIO_BBB::pinMode(uint8_t pin, uint8_t output)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (output == HAL_GPIO_INPUT) {
        *gpio_bank[bank].oe |= (1U<<bankpin);
    } else {
        *gpio_bank[bank].oe &= ~(1U<<bankpin);
    }
}

uint8_t GPIO_BBB::read(uint8_t pin) {

    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return 0;
    }
    return *gpio_bank[bank].in & (1U<<bankpin) ? HIGH : LOW;

}

void GPIO_BBB::write(uint8_t pin, uint8_t value)
{
    uint8_t bank = pin/32;
    uint8_t bankpin = pin & 0x1F;
    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (value == LOW) {
        *gpio_bank[bank].out &= ~(1U<<bankpin);
    } else {
        *gpio_bank[bank].out |= 1U<<bankpin;
    }
}

void GPIO_BBB::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_BBB::channel(uint16_t n) {
    return new DigitalSource(n);
}

bool GPIO_BBB::usb_connected(void)
{
    return false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF ||
       // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD ||
       // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI ||
       // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE ||
       // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
