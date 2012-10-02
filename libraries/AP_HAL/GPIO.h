
#ifndef __AP_HAL_GPIO_H__
#define __AP_HAL_GPIO_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

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
    virtual AP_HAL::DigitalSource* channel(int n) = 0;
};

#endif // __AP_HAL_GPIO_H__
