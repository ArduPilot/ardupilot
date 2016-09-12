#pragma once

#include "AP_HAL_SITL.h"

class HALSITL::GPIO : public AP_HAL::GPIO {
public:
    GPIO(SITL_State *_sitlState) {
        sitlState = _sitlState;
    }
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);

private:
    SITL_State *sitlState;
};

class HALSITL::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t _pin);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();

private:
    uint8_t pin;
};
