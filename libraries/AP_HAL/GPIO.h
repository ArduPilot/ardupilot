
#ifndef __AP_HAL_GPIO_H__
#define __AP_HAL_GPIO_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"

#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_INTERRUPT_LOW 0
#define HAL_GPIO_INTERRUPT_HIGH 1
#define HAL_GPIO_INTERRUPT_FALLING 2
#define HAL_GPIO_INTERRUPT_RISING 3

class AP_HAL::DigitalSource {
public:
    virtual void    mode(uint8_t output) = 0;
    virtual uint8_t read() = 0;
    virtual void    write(uint8_t value) = 0;
    virtual void    toggle() = 0;
};

class AP_HAL::GPIO {
public:
    /* Implementations that use virtual pin should fill this enum with the
     * names for the pins and also implement the functions that use virtual
     * pins, i.e., those that receive an enum VirtualPin instead of uint8_t */
    enum VirtualPin : uint8_t;

    GPIO() {}
    virtual void    init() = 0;

    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;
    virtual void    pinMode(enum VirtualPin pin, uint8_t output) {}

    virtual uint8_t read(uint8_t pin) = 0;
    virtual uint8_t read(enum VirtualPin pin) { return 0; }

    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    write(enum VirtualPin pin, uint8_t value) {}

    virtual void    toggle(uint8_t pin) = 0;
    virtual void    toggle(enum VirtualPin pin) {}

    virtual int8_t  analogPinToDigitalPin(uint8_t pin) = 0;
    virtual int8_t  analogPinToDigitalPin(enum VirtualPin pin) { return -1; }

    /* Alternative interface: */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;
    virtual AP_HAL::DigitalSource* channel(enum VirtualPin n) { return nullptr; }

    /* Interrupt interface: */
    virtual bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
                                     uint8_t mode) = 0;
    virtual bool    attach_interrupt(enum VirtualPin interrupt_num,
                                     AP_HAL::Proc p,
                                     uint8_t mode) { return false; }

    /* return true if USB cable is connected */
    virtual bool    usb_connected(void) = 0;
};

#endif // __AP_HAL_GPIO_H__
