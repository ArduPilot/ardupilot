#pragma once

#include <AP_HAL/HAL.h>

#if HAL_LINUX_GPIO_PILOTPI_ENABLED

#include "GPIO_RPI.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

/**
 * Provide unused Raspberry PI pins as GPIOs in the order of the PCB pinheader:
 * AP GPIO Pin      RPI BCM GPIO 
 * =============================
 * 101              4
 * 102              14
 * 103              17
 * 104              27
 * 105              22
 * 106              23
 * 107              7
 * 108              5
 * 109              6
 * 110              12
 * 111              13
 * 112              16
 * 113              26
 *
 * and those SERVO outputs that are configured as GPIO:
 * AP GPIO Pin      SERVO
 * ==============================
 * 50               0
 * 51               1
 * ...
 */
class GPIO_PilotPi : public GPIO_RPI
{

public:
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;

private:
    uint8_t PilotPiGPIOS[13] = {
        RPI_GPIO_<4>(),
        RPI_GPIO_<14>(),
        RPI_GPIO_<17>(),
        RPI_GPIO_<27>(),
        RPI_GPIO_<22>(),
        RPI_GPIO_<23>(),
        RPI_GPIO_<7>(),
        RPI_GPIO_<5>(),
        RPI_GPIO_<6>(),
        RPI_GPIO_<12>(),
        RPI_GPIO_<13>(),
        RPI_GPIO_<16>(),
        RPI_GPIO_<26>()
    };
    bool mainOutPin(uint8_t ap_pin, uint8_t *pin);
    bool auxOutPin(uint8_t ap_pin, uint8_t *pin);
};

#endif  // HAL_LINUX_GPIO_PILOTPI_ENABLED
