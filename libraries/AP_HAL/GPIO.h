
#ifndef __AP_HAL_GPIO_H__
#define __AP_HAL_GPIO_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_FALLING 2
#define GPIO_RISING 3

class AP_HAL::DigitalSource {
public:
    virtual void    mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void    write(uint8_t value) = 0; 
};

class AP_HAL::GPIO {
public:
    GPIO() {}
    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;
    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    /* Interrupt interface: */
    virtual bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode) = 0;
};

#endif // __AP_HAL_GPIO_H__
