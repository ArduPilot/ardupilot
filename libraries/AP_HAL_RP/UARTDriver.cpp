#include "UARTDriver.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

// Global pointers for interrupt handling (simplified)
static UARTDriver* uart_drivers[2] = {nullptr, nullptr};

extern "C" {
    void uart0_irq_handler() { if (uart_drivers[0]) uart_drivers[0]->handle_irq(); }
    void uart1_irq_handler() { if (uart_drivers[1]) uart_drivers[1]->handle_irq(); }
}

UARTDriver::UARTDriver(uart_inst_t *uart_hw, uint8_t tx_pin, uint8_t rx_pin, uint8_t instance) :
    _uart(uart_hw),
    _tx_pin(tx_pin),
    _rx_pin(rx_pin),
    _instance(instance),
    _initialized(false), 
    _readbuf{UART_RX_BUFFER_SIZE},
    _writebuf{UART_TX_BUFFER_SIZE},
    _write_mutex{}
{}

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) {
    if (_initialized) _end();

    _baudrate = baud;
    _readbuf.set_size(rxSpace >= 128 ? rxSpace : 128);
    _writebuf.set_size(txSpace >= 128 ? txSpace : 128);

    uart_init(_uart, _baudrate);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);

    // Apply inversion and other options if they were set earlier
    set_options(_last_options);

    // Configure interrupts
    uart_drivers[_instance] = this;
    int irq_num = (_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irq_num, (_uart == uart0) ? uart0_irq_handler : uart1_irq_handler);
    irq_set_enabled(irq_num, true);

    // Enable FIFO and receive interrupts (RX)
    uart_set_fifo_enabled(_uart, true);
    uart_set_irq_enables(_uart, true, false);

    _initialized = true;
}

void UARTDriver::_end() {
    _initialized = false;
    int irq_num = (_uart == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_enabled(irq_num, false);
    uart_deinit(_uart);
}

void UARTDriver::handle_irq() {
    // Read everything in the hardware FIFO into the ArduPilot buffer
    while (uart_is_readable(_uart)) {
        uint8_t ch = uart_getc(_uart);
        if (!_readbuf.write(&ch, 1)) {
            // overflow: dropped bytes
        }
    }
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size) {
    if (!_initialized) return 0;
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t count) {
    if (!_initialized) return -1;
    return (ssize_t)_readbuf.read(buffer, count);
}

uint32_t UARTDriver::_available() {
    return _readbuf.available();
}

bool UARTDriver::tx_pending() {
    return (_writebuf.available() > 0) || (uart_get_hw(_uart)->fr & UART_UARTFR_BUSY_BITS);
}

void UARTDriver::_timer_tick() {
    if (!_initialized) return;

    // Push data from the software buffer into the hardware FIFO
    WITH_SEMAPHORE(_write_mutex);
    while (uart_is_writable(_uart) && _writebuf.available() > 0) {
        uint8_t ch;
        if (_writebuf.read(&ch, 1)) {
            uart_putc(_uart, ch);
        }
    }
}

void UARTDriver::_flush() {
    while (tx_pending()) {
        _timer_tick();
    }
}

bool UARTDriver::_discard_input(void) {
    _readbuf.clear();
    return true;
}

bool UARTDriver::set_options(uint16_t options) {
    // Store options in the base class
    _last_options = options;

    if (!_initialized) {
        return true; 
    }

    // Define flags from AP_HAL/UARTDriver.h
    bool invert_tx = (options & OPTION_TXINV);
    bool invert_rx = (options & OPTION_RXINV);
    bool swap_rx_tx = (options & OPTION_SWAP);

    // Inversion processing via GPIO controller RP2350
    // GPIO_OVERRIDE_INVERT inverts the logic level on the input/output
    if (invert_tx) {
        gpio_set_outover(_tx_pin, GPIO_OVERRIDE_INVERT);
    } else {
        gpio_set_outover(_tx_pin, GPIO_OVERRIDE_NORMAL);
    }

    if (invert_rx) {
        gpio_set_inover(_rx_pin, GPIO_OVERRIDE_INVERT);
    } else {
        gpio_set_inover(_rx_pin, GPIO_OVERRIDE_NORMAL);
    }

    // Swap processing (if you need to swap pins)
    if (swap_rx_tx) {
        gpio_set_function(_tx_pin, GPIO_FUNC_UART); // Now it's TX
        gpio_set_function(_rx_pin, GPIO_FUNC_UART); // Now it's RX
        // Note: In RP2350, UART pins are hard-wired to functions,
        // so true SWAP only works if the selected pins support UART_TX/RX cross-multiplexing.
    }

    // Half-Duplex processing (if option is enabled)
    if (options & OPTION_HDPLEX) {
        // In half-duplex mode, we usually connect TX and RX
        // On RP2350, this is often implemented through a single pin in Open-Drain mode
        gpio_set_oeover(_tx_pin, GPIO_OVERRIDE_NORMAL);
    }

    return true;
}

void UARTDriver::configure_parity(uint8_t v) {
    uart_parity_t p = UART_PARITY_NONE;
    if (v == 1) p = UART_PARITY_ODD;
    if (v == 2) p = UART_PARITY_EVEN;
    uart_set_format(_uart, 8, 1, p);
}

void UARTDriver::set_stop_bits(int n) {
    uart_set_format(_uart, 8, n, UART_PARITY_NONE);
}

uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes) {
    uint64_t byte_time_us = 10000000ULL / _baudrate; // 10 bits per byte
    return AP_HAL::micros64() - (nbytes * byte_time_us);
}

uint32_t UARTDriver::txspace() {
    if (!_initialized) {
        return 0;
    }
    return _writebuf.space();
}

void UARTDriver::vprintf(const char *fmt, va_list ap) {
    if (!_initialized) {
        return;
    }
    // Temporary buffer for rendering the string.
    char buffer[128];
    
    // Format the string
    int n = vsnprintf(buffer, sizeof(buffer), fmt, ap);
    if (n > 0) {
        // Limit the length if the string is longer than the buffer
        size_t len = (size_t)n;
        if (len >= sizeof(buffer)) {
            len = sizeof(buffer) - 1;
        }
        // Call your internal write method
        _write((const uint8_t *)buffer, len);
    }
}
