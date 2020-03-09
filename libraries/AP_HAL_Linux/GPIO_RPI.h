#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"

#define LOW                 0
#define HIGH                1

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

/**
 * @brief Check for valid Raspberry Pi pin range
 *
 * @tparam pin
 * @return uint8_t
 */
template <uint8_t pin> constexpr uint8_t RPI_GPIO_()
{
    static_assert(pin > 1 && pin < 32, "Invalid pin value.");
    return pin;
}

namespace Linux {

class GPIO_RPI : public AP_HAL::GPIO {
public:
    GPIO_RPI();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

private:
    volatile uint32_t *_gpio;
};

}
