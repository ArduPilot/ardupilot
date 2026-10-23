#include <AP_HAL/AP_HAL.h>

#include "GPIO_RPI_RP1.h"

#include <fcntl.h>
#include <poll.h>
#include <cstdlib>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace Linux;

using namespace AP_HAL;

GPIO_RPI_RP1::GPIO_RPI_RP1() {
}

bool GPIO_RPI_RP1::openMemoryDevice() {
    _system_memory_device = open(PATH_DEV_GPIOMEM, O_RDWR | O_SYNC | O_CLOEXEC);
    if (_system_memory_device < 0) {
        AP_HAL::panic("Can't open %s", PATH_DEV_GPIOMEM);
        return false;
    }
    return true;
}

void GPIO_RPI_RP1::closeMemoryDevice() {
    close(_system_memory_device);
    _system_memory_device = -1;
}

volatile uint32_t* GPIO_RPI_RP1::get_memory_pointer(uint32_t address, uint32_t length) const {
    auto pointer = mmap(
        nullptr,
        length,
        PROT_READ | PROT_WRITE | PROT_EXEC,
        MAP_SHARED | MAP_LOCKED,  // Shared with other processes
        _system_memory_device,    // File to map
        address                   // Offset to GPIO peripheral
    );

    if (pointer == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<volatile uint32_t*>(pointer);
}

void GPIO_RPI_RP1::init() {
    if (!openMemoryDevice()) {
        AP_HAL::panic("Failed to initialize memory device.");
        return;
    }

    _gpio = get_memory_pointer(IO_BANK0_OFFSET, MEM_SIZE);
    if (!_gpio) {
        AP_HAL::panic("Failed to get GPIO memory map.");
    }

    closeMemoryDevice();
}

uint32_t GPIO_RPI_RP1::read_register(uint32_t offset) const {
    return _gpio[offset];
}

void GPIO_RPI_RP1::write_register(uint32_t offset, uint32_t value) {
    _gpio[offset] = value;
}

GPIO_RPI_RP1::Mode GPIO_RPI_RP1::direction(uint8_t pin) const {
    uint32_t offset = (SYS_RIO0_OFFSET + RIO_OE) / REG_SIZE;
    uint32_t value = (read_register(offset) >> pin) & 0b1;
    return value > 0 ? Mode::Output : Mode::Input;
}

void GPIO_RPI_RP1::set_direction(uint8_t pin, Mode mode) {
    uint32_t offset = (SYS_RIO0_OFFSET + RIO_OE) / REG_SIZE;
    offset += (mode == Mode::Output ? SET_OFFSET : CLR_OFFSET) / REG_SIZE;
    write_register(offset, 1 << pin);
}

void GPIO_RPI_RP1::input_enable(uint8_t pin) {
    uint32_t offset = (PADS_BANK0_OFFSET + PADS_GPIO + pin * PADS_OFFSET + SET_OFFSET) / REG_SIZE;
    write_register(offset, PADS_IN_ENABLE_MASK);
}

void GPIO_RPI_RP1::input_disable(uint8_t pin) {
    uint32_t offset = (PADS_BANK0_OFFSET + PADS_GPIO + pin * PADS_OFFSET + CLR_OFFSET) / REG_SIZE;
    write_register(offset, PADS_IN_ENABLE_MASK);
}

void GPIO_RPI_RP1::output_enable(uint8_t pin) {
    uint32_t offset = (PADS_BANK0_OFFSET + PADS_GPIO + pin * PADS_OFFSET + CLR_OFFSET) / REG_SIZE;
    write_register(offset, PADS_OUT_DISABLE_MASK);
}

void GPIO_RPI_RP1::output_disable(uint8_t pin) {
    uint32_t offset = (PADS_BANK0_OFFSET + PADS_GPIO + pin * PADS_OFFSET + SET_OFFSET) / REG_SIZE;
    write_register(offset, PADS_OUT_DISABLE_MASK);
}

void GPIO_RPI_RP1::pinMode(uint8_t pin, uint8_t mode) {
    if (mode == HAL_GPIO_INPUT) {
        input_enable(pin);
        set_direction(pin, Mode::Input);
        set_mode(pin, Mode::Input);
    } else if (mode == HAL_GPIO_OUTPUT) {
        output_enable(pin);
        set_direction(pin, Mode::Output);
        set_mode(pin, Mode::Input);
    }
}

void GPIO_RPI_RP1::pinMode(uint8_t pin, uint8_t mode, uint8_t alt) {
    if (mode == HAL_GPIO_ALT) {
        set_mode(pin, static_cast<Mode>(alt));
        return;
    }
    pinMode(pin, mode);
}

void GPIO_RPI_RP1::set_mode(uint8_t pin, Mode mode) {
    FunctionSelect alt_value;

    switch (mode) {
        // ALT5 is used for GPIO, check datasheet pinout alternative functions
        case Mode::Alt5:
        case Mode::Input:
        case Mode::Output:
            alt_value = FunctionSelect::Alt5;
            break;
        case Mode::Alt0: alt_value = FunctionSelect::Alt0; break;
        case Mode::Alt1: alt_value = FunctionSelect::Alt1; break;
        case Mode::Alt2: alt_value = FunctionSelect::Alt2; break;
        case Mode::Alt3: alt_value = FunctionSelect::Alt3; break;
        case Mode::Alt4: alt_value = FunctionSelect::Alt4; break;
        case Mode::Alt6: alt_value = FunctionSelect::Alt6; break;
        case Mode::Alt7: alt_value = FunctionSelect::Alt7; break;
        case Mode::Alt8: alt_value = FunctionSelect::Alt8; break;
        default: alt_value = FunctionSelect::Null; break;
    }

    uint32_t ctrl_offset = (IO_BANK0_OFFSET + GPIO_CTRL + (pin * GPIO_OFFSET) + RW_OFFSET) / REG_SIZE;
    uint32_t reg_val = read_register(ctrl_offset);
    reg_val &= ~CTRL_FUNCSEL_MASK; // Clear existing function select bits
    reg_val |= (static_cast<uint32_t>(alt_value) << CTRL_FUNCSEL_LSB);
    write_register(ctrl_offset, reg_val);

}

void GPIO_RPI_RP1::set_pull(uint8_t pin, PadsPull mode) {
    uint32_t pads_offset = (PADS_BANK0_OFFSET + PADS_GPIO + (pin * PADS_OFFSET) + RW_OFFSET) / REG_SIZE;
    uint32_t reg_val = read_register(pads_offset);
    reg_val &= ~PADS_PULL_MASK; // Clear existing bias bits
    reg_val |= (static_cast<uint8_t>(mode) << PADS_PULL_LSB);
    write_register(pads_offset, reg_val);
}

uint8_t GPIO_RPI_RP1::read(uint8_t pin) {
    uint32_t in_offset = (SYS_RIO0_OFFSET + RIO_IN) / REG_SIZE;
    uint32_t reg_val = read_register(in_offset);
    return (reg_val >> pin) & 0x1;
}

void GPIO_RPI_RP1::write(uint8_t pin, uint8_t value) {
    uint32_t register_offset = value ? SET_OFFSET : CLR_OFFSET;
    uint32_t offset = (SYS_RIO0_OFFSET + RIO_OUT + register_offset) / REG_SIZE;
    write_register(offset, 1 << pin);
}

void GPIO_RPI_RP1::toggle(uint8_t pin) {
    uint32_t flag = (1 << pin);
    _gpio_output_port_status ^= flag;
    write(pin, (_gpio_output_port_status & flag) >> pin);
}
