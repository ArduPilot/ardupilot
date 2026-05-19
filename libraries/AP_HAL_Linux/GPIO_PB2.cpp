#include <AP_HAL/AP_HAL.h>

#if HAL_LINUX_GPIO_PB2_ENABLED

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

#define LOW     0
#define HIGH    1

using namespace Linux;

GPIO_PB2::GPIO_PB2()
{}

void GPIO_PB2::init()
{
    int mem_fd;
    
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0) {
            printf("can't open /dev/mem \n");
            exit (-1);
    }

    off_t offsets[LINUX_GPIO_NUM_BANKS] = { GPIO1_BASE, GPIO0_BASE, GPIO0_BASE, GPIO1_BASE, GPIO0_BASE, GPIO0_BASE};
    uint32_t gpio_dir[LINUX_GPIO_NUM_BANKS] = {GPIO_DIR01, GPIO_DIR01, GPIO_DIR23, GPIO_DIR23, GPIO_DIR45, GPIO_DIR45};
    uint32_t gpio_set[LINUX_GPIO_NUM_BANKS] = {GPIO_SET_DATA01, GPIO_SET_DATA01, GPIO_SET_DATA23, GPIO_SET_DATA23, GPIO_SET_DATA45, GPIO_SET_DATA45};
    uint32_t gpio_out[LINUX_GPIO_NUM_BANKS] = {GPIO_OUT_DATA01, GPIO_OUT_DATA01, GPIO_OUT_DATA23, GPIO_OUT_DATA23, GPIO_OUT_DATA45, GPIO_OUT_DATA45};
    uint32_t gpio_clr[LINUX_GPIO_NUM_BANKS] = {GPIO_CLR_DATA01, GPIO_CLR_DATA01, GPIO_CLR_DATA23, GPIO_CLR_DATA23, GPIO_CLR_DATA45, GPIO_CLR_DATA45};
    uint32_t gpio_in[LINUX_GPIO_NUM_BANKS] = {GPIO_IN_DATA01, GPIO_IN_DATA01, GPIO_IN_DATA23, GPIO_IN_DATA23, GPIO_IN_DATA45, GPIO_IN_DATA45};

    for (uint8_t i=0; i<LINUX_GPIO_NUM_BANKS; i++) {
        gpio_bank[i].gpio_base = (volatile unsigned *)mmap(0, GPIO_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, offsets[i]);
        if ((char *)gpio_bank[i].gpio_base == MAP_FAILED) {
            AP_HAL::panic("unable to map GPIO bank");
        }

        gpio_bank[i].gpio_direction = gpio_bank[i].gpio_base + gpio_dir[i];
        gpio_bank[i].gpio_set = gpio_bank[i].gpio_base + gpio_set[i];
        gpio_bank[i].gpio_clear = gpio_bank[i].gpio_base + gpio_clr[i];
        gpio_bank[i].gpio_out = gpio_bank[i].gpio_base + gpio_out[i];
        gpio_bank[i].gpio_in = gpio_bank[i].gpio_base + gpio_in[i];
    }

    close(mem_fd);
}

void GPIO_PB2::pinMode(uint8_t pin, uint8_t output) {

    uint8_t bank = pin/16;
    uint8_t bankpin = 0;

    if(pin < 64)
        bankpin = pin & 0xF;
    else
        bankpin = pin & 0x1F;

    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (output == HAL_GPIO_INPUT) {
        *gpio_bank[bank].gpio_direction |= (1U<<bankpin);
    } else {
        *gpio_bank[bank].gpio_direction &= ~(1U<<bankpin);
    }
}

uint8_t GPIO_PB2::read(uint8_t pin) {

    uint8_t bank = pin/16;
    uint8_t bankpin = 0;

    if(pin < 64)
        bankpin = pin & 0xF;
    else
        bankpin = pin & 0x1F;

    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return 0;
    }
    
    return *gpio_bank[bank].gpio_in & (1U<<bankpin) ? HIGH : LOW;
}

void GPIO_PB2::write(uint8_t pin, uint8_t value) {

    uint8_t bank = pin/16;
    uint8_t bankpin = 0;
    
    if(pin < 64)
        bankpin = pin & 0xF;
    else
        bankpin = pin & 0x1F;

    if (bank >= LINUX_GPIO_NUM_BANKS) {
        return;
    }
    if (value == LOW) {
        *gpio_bank[bank].gpio_clear |= 1U<<bankpin;
        *gpio_bank[bank].gpio_out   &= ~(1U<<bankpin);
    } else {
        *gpio_bank[bank].gpio_set   |= 1U<<bankpin;
        *gpio_bank[bank].gpio_out   |= 1U<<bankpin;
    }
}

void GPIO_PB2::toggle(uint8_t pin) {

    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_PB2::channel(uint16_t n) {
    return NEW_NOTHROW DigitalSource(n);
}

bool GPIO_PB2::usb_connected(void)
{
    return false;
}

#endif  // HAL_LINUX_GPIO_PB2_ENABLED