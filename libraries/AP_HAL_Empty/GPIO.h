#pragma once

#include "AP_HAL_Empty.h"

class Empty::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num,
                             irq_handler_fn_t fn,
                             INTERRUPT_TRIGGER_TYPE mode) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void);
};

class Empty::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t v);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    uint8_t _v;
};
