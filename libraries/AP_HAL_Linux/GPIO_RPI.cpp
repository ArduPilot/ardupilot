#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DARK || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR || \
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "GPIO.h"
#include "Util_RPI.h"

#define GPIO_RPI_MAX_NUMBER_PINS 32

using namespace Linux;

extern const AP_HAL::HAL& hal;

// Range based in the first memory address of the first register and the last memory addres
// for the GPIO section (0x7E20'00B4 - 0x7E20'0000).
const uint8_t GPIO_RPI::_gpio_registers_memory_range = 0xB4;
const char* GPIO_RPI::_system_memory_device_path = "/dev/mem";

GPIO_RPI::GPIO_RPI()
{
}

void GPIO_RPI::set_gpio_mode_alt(int pin, int alternative)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    /** Creates a mask to enable the alternative function based in the following logic:
     *
     * | Alternative Function | 3 bits value |
     * |:--------------------:|:------------:|
     * |      Function 0      |     0b100    |
     * |      Function 1      |     0b101    |
     * |      Function 2      |     0b110    |
     * |      Function 3      |     0b111    |
     * |      Function 4      |     0b011    |
     * |      Function 5      |     0b010    |
     */
    const uint8_t alternative_value =
        (alternative < 4 ? (alternative + 4) : (alternative == 4 ? 3 : 2));
    // 0b00'000'000'000'000'000'000'ALT'000'000'000 enables alternative for the 4th pin
    const uint32_t mask_with_alt = static_cast<uint32_t>(alternative_value) << tree_bits_position_in_register;
    const uint32_t mask = 0b111 << tree_bits_position_in_register;
    // Clear all bits in our position and apply our mask with alt values
    uint32_t register_value = _gpio[pin / pins_per_register];
    register_value &= ~mask;
    _gpio[pin / pins_per_register] = register_value | mask_with_alt;
}

void GPIO_RPI::set_gpio_mode_in(int pin)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    // Create a mask that only removes the bits in this specific GPIO pin, E.g:
    // 0b11'111'111'111'111'111'111'000'111'111'111 for the 4th pin
    const uint32_t mask = ~(0b111<<tree_bits_position_in_register);
    // Apply mask
    _gpio[pin / pins_per_register] &= mask;
}

void GPIO_RPI::set_gpio_mode_out(int pin)
{
    // Each register can contain 10 pins
    const uint8_t pins_per_register = 10;
    // Calculates the position of the 3 bit mask in the 32 bits register
    const uint8_t tree_bits_position_in_register = (pin%pins_per_register)*3;
    // Create a mask to enable the bit that sets output functionality
    // 0b00'000'000'000'000'000'000'001'000'000'000 enables output for the 4th pin
    const uint32_t mask_with_bit = 0b001 << tree_bits_position_in_register;
    const uint32_t mask = 0b111 << tree_bits_position_in_register;
    // Clear all bits in our position and apply our mask with alt values
    uint32_t register_value = _gpio[pin / pins_per_register];
    register_value &= ~mask;
    _gpio[pin / pins_per_register] = register_value | mask_with_bit;
}

void GPIO_RPI::set_gpio_high(int pin)
{
    // Calculate index of the array for the register GPSET0 (0x7E20'001C)
    constexpr uint32_t gpset0_memory_offset_value = 0x1c;
    constexpr uint32_t gpset0_index_value = gpset0_memory_offset_value / sizeof(*_gpio);
    _gpio[gpset0_index_value] = 1 << pin;
}

void GPIO_RPI::set_gpio_low(int pin)
{
    // Calculate index of the array for the register GPCLR0 (0x7E20'0028)
    constexpr uint32_t gpclr0_memory_offset_value = 0x28;
    constexpr uint32_t gpclr0_index_value = gpclr0_memory_offset_value / sizeof(*_gpio);
    _gpio[gpclr0_index_value] = 1 << pin;
}

bool GPIO_RPI::get_gpio_logic_state(int pin)
{
    // Calculate index of the array for the register GPLEV0 (0x7E20'0034)
    constexpr uint32_t gplev0_memory_offset_value = 0x34;
    constexpr uint32_t gplev0_index_value = gplev0_memory_offset_value / sizeof(*_gpio);
    return _gpio[gplev0_index_value] & (1 << pin);
}

uint32_t GPIO_RPI::get_address(GPIO_RPI::Address address, GPIO_RPI::PeripheralOffset offset) const
{
    return static_cast<uint32_t>(address) + static_cast<uint32_t>(offset);
}

volatile uint32_t* GPIO_RPI::get_memory_pointer(uint32_t address, uint32_t range) const
{
    auto pointer = mmap(
        nullptr,                         // Any adddress in our space will do
        range,                           // Map length
        PROT_READ|PROT_WRITE|PROT_EXEC,  // Enable reading & writing to mapped memory
        MAP_SHARED|MAP_LOCKED,           // Shared with other processes
        _system_memory_device,           // File to map
        address                          // Offset to GPIO peripheral
    );

    if (pointer == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<volatile uint32_t*>(pointer);
}

bool GPIO_RPI::openMemoryDevice()
{
    _system_memory_device = open(_system_memory_device_path, O_RDWR|O_SYNC|O_CLOEXEC);
    if (_system_memory_device < 0) {
        AP_HAL::panic("Can't open %s", GPIO_RPI::_system_memory_device_path);
        return false;
    }

    return true;
}

void GPIO_RPI::closeMemoryDevice()
{
    close(_system_memory_device);
    // Invalidate device variable
    _system_memory_device = -1;
}

void GPIO_RPI::init()
{
    const LINUX_BOARD_TYPE rpi_version = UtilRPI::from(hal.util)->detect_linux_board_type();

    GPIO_RPI::Address peripheral_base;
    if(rpi_version == LINUX_BOARD_TYPE::RPI_ZERO_1) {
        peripheral_base = Address::BCM2708_PERIPHERAL_BASE;
    } else if (rpi_version == LINUX_BOARD_TYPE::RPI_2_3_ZERO2) {
        peripheral_base = Address::BCM2709_PERIPHERAL_BASE;
    } else if (rpi_version == LINUX_BOARD_TYPE::RPI_4) {
        peripheral_base = Address::BCM2711_PERIPHERAL_BASE;
    } else {
        AP_HAL::panic("Unknown rpi_version, cannot locate peripheral base address");
        return;
    }

    if (!openMemoryDevice()) {
        AP_HAL::panic("Failed to initialize memory device.");
        return;
    }

    const uint32_t gpio_address = get_address(peripheral_base, PeripheralOffset::GPIO);

    _gpio = get_memory_pointer(gpio_address, _gpio_registers_memory_range);
    if (!_gpio) {
        AP_HAL::panic("Failed to get GPIO memory map.");
    }

    // No need to keep mem_fd open after mmap
    closeMemoryDevice();
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output)
{
    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(pin);
    } else {
        set_gpio_mode_in(pin);
        set_gpio_mode_out(pin);
    }
}

void GPIO_RPI::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(pin);
    } else if (output == HAL_GPIO_ALT) {
        set_gpio_mode_in(pin);
        set_gpio_mode_alt(pin, alt);
    } else {
        set_gpio_mode_in(pin);
        set_gpio_mode_out(pin);
    }
}

uint8_t GPIO_RPI::read(uint8_t pin)
{
    if (pin >= GPIO_RPI_MAX_NUMBER_PINS) {
        return 0;
    }
    return static_cast<uint8_t>(get_gpio_logic_state(pin));
}

void GPIO_RPI::write(uint8_t pin, uint8_t value)
{
    if (value != 0) {
        set_gpio_high(pin);
    } else {
        set_gpio_low(pin);
    }
}

void GPIO_RPI::toggle(uint8_t pin)
{
    if (pin >= GPIO_RPI_MAX_NUMBER_PINS) {
        return ;
    }
    uint32_t flag = (1 << pin);
    _gpio_output_port_status ^= flag;
    write(pin, (_gpio_output_port_status & flag) >> pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_RPI::channel(uint16_t n)
{
    return new DigitalSource(n);
}

bool GPIO_RPI::usb_connected(void)
{
    return false;
}

#endif
