#pragma once
#include "GPIO_RPI.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

class GPIO_Navigator : public GPIO_RPI
{

public:
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
private:
    uint8_t AllowedGPIOS[1] = {RPI_GPIO_<26>()};
    bool    pinAllowed(uint8_t pin);
};


