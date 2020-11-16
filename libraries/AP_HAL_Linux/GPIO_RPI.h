#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"

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

/**
 * @brief Class for Raspberry PI GPIO control
 *
 *  For more information: https://elinux.org/RPi_BCM2835_GPIOs
 *
 */
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
    // Raspberry Pi BASE memory address
    enum class Address : uint32_t {
        BCM2708_PERIPHERAL_BASE = 0x20000000, // Raspberry Pi 0/1
        BCM2709_PERIPHERAL_BASE = 0x3F000000, // Raspberry Pi 2/3
        BCM2711_PERIPHERAL_BASE = 0xFE000000, // Raspberry Pi 4
    };

    // Offset between peripheral base address
    enum class PeripheralOffset : uint32_t {
        GPIO = 0x200000,
    };

    /**
     * @brief Open memory device to allow gpio address access
     *  Should be used before get_memory_pointer calls in the initialization
     *
     * @return true
     * @return false
     */
    bool openMemoryDevice();

    /**
     * @brief Close open memory device
     *
     */
    void closeMemoryDevice();

    /**
     * @brief Return pointer to memory location with specific range access
     *
     * @param address
     * @param range
     * @return volatile uint32_t*
     */
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;

    /**
     * @brief Get memory address based in base address and peripheral offset
     *
     * @param address
     * @param offset
     * @return uint32_t
     */
    uint32_t get_address(GPIO_RPI::Address address, GPIO_RPI::PeripheralOffset offset) const;

    /**
     * @brief Change functionality of GPIO Function Select Registers (GPFSELn) to any alternative function.
     * Each GPIO pin is mapped to 3 bits inside a 32 bits register, E.g:
     *
     * 0b00...'010'101
     *   ││    │││ ││└── GPIO Pin N, 1st bit, LSBit
     *   ││    │││ │└─── GPIO Pin N, 2nd bit
     *   ││    │││ └──── GPIO Pin N, 3rd bit, MSBit
     *   ││    ││└────── GPIO Pin N+1, 1st bit, LSBit
     *   ││    │└─────── GPIO Pin N+1, 2nd bit,
     *   ││    └──────── GPIO Pin N+1, 3rd bit, MSBit
     *   ││   ...
     *   │└───────────── Reserved
     *   └────────────── Reserved
     *
     * And the value of this 3 bits selects the functionality of the GPIO pin, E.g:
     *  000 = GPIO Pin N is an input
     *  001 = GPIO Pin N is an output
     *  100 = GPIO Pin N takes alternate function 0
     *  101 = GPIO Pin N takes alternate function 1
     *  110 = GPIO Pin N takes alternate function 2
     *  111 = GPIO Pin N takes alternate function 3
     *  011 = GPIO Pin N takes alternate function 4
     *  010 = GPIO Pin N takes alternate function 5
     *
     * The alternative functions are defined in the BCM datasheet under "Alternative Function"
     * section for each pin.
     *
     * This information is also valid for:
     *  - Linux::GPIO_RPI::set_gpio_mode_in
     *  - Linux::GPIO_RPI::set_gpio_mode_out
     *
     * @param pin
     */
    void set_gpio_mode_alt(int pin, int alternative);

    /**
     * @brief Set a specific GPIO as input
     * Check Linux::GPIO_RPI::set_gpio_mode_alt for more information.
     *
     * @param pin
     */
    void set_gpio_mode_in(int pin);

    /**
     * @brief Set a specific GPIO as output
     * Check Linux::GPIO_RPI::set_gpio_mode_alt for more information.
     *
     * @param pin
     */
    void set_gpio_mode_out(int pin);

    /**
     * @brief Modify GPSET0 (GPIO Pin Output Set 0) register to set pin as high
     * Writing zero to this register has no effect, please use Linux::GPIO_RPI::set_gpio_low
     * to set pin as low.
     *
     * GPSET0 is a 32bits register that each bit points to a respective GPIO pin:
     * 0b...101
     *      ││└── GPIO Pin 1, 1st bit, LSBit, defined as High
     *      │└─── GPIO Pin 2, 2nd bit, No effect
     *      └──── GPIO Pin 3, 3rd bit, defined as High
     *     ...
     *
     * @param pin
     */
    void set_gpio_high(int pin);

    /**
     * @brief Modify GPCLR0 (GPIO Pin Output Clear 0) register to set pin as low
     * Writing zero to this register has no effect, please use Linux::GPIO_RPI::set_gpio_high
     * to set pin as high.
     *
     * GPCLR0 is a 32bits register that each bit points to a respective GPIO pin:
     * 0b...101
     *      ││└── GPIO Pin 1, 1st bit, LSBit, defined as Low
     *      │└─── GPIO Pin 2, 2nd bit, No effect
     *      └──── GPIO Pin 3, 3rd bit, defined as Low
     *
     * @param pin
     */
    void set_gpio_low(int pin);

    /**
     * @brief Read GPLEV0 (GPIO Pin Level 0) register check the logic state of a specific pin
     *
     * GPLEV0 is a 32bits register that each bit points to a respective GPIO pin:
     * 0b...101
     *      ││└── GPIO Pin 1, 1st bit, LSBit, Pin is in High state
     *      │└─── GPIO Pin 2, 2nd bit, Pin is in Low state
     *      └──── GPIO Pin 3, 3rd bit, Pin is in High state
     *
     * @param pin
     * @return true
     * @return false
     */
    bool get_gpio_logic_state(int pin);

    // Memory pointer to gpio registers
    volatile uint32_t* _gpio;
    // Memory range for the gpio registers
    static const uint8_t _gpio_registers_memory_range;
    // Path to memory device (E.g: /dev/mem)
    static const char* _system_memory_device_path;
    // File descriptor for the memory device file
    // If it's negative, then there was an error opening the file.
    int _system_memory_device;
};

}
