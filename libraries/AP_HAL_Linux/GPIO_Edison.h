#pragma once

#include <stdint.h>
#include <mraa.h>

#include "AP_HAL_Linux.h"

#define LOW                 0
#define HIGH                1

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

namespace Linux {

class GPIO_Edison : public AP_HAL::GPIO {
public:
    GPIO_Edison();
    mraa_gpio_context gpio;
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

private:
    volatile uint32_t *_gpio;

};

}
