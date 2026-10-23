#pragma once

class GPIO_RPI_HAL {
public:
    GPIO_RPI_HAL() {}
    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    toggle(uint8_t pin) = 0;
};
