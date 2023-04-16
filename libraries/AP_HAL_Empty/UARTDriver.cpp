
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

Empty::UARTDriver::UARTDriver() {}

void Empty::UARTDriver::begin(uint32_t b) {}
void Empty::UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void Empty::UARTDriver::end() {}
void Empty::UARTDriver::flush() {}
bool Empty::UARTDriver::is_initialized() { return false; }
void Empty::UARTDriver::set_blocking_writes(bool blocking) {}
bool Empty::UARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
uint32_t Empty::UARTDriver::available() { return 0; }
uint32_t Empty::UARTDriver::txspace() { return 1; }
bool Empty::UARTDriver::read(uint8_t &b) { return false; }
bool Empty::UARTDriver::discard_input() { return false; }

/* Empty implementations of Print virtual methods */
size_t Empty::UARTDriver::write(uint8_t c) { return 0; }

size_t Empty::UARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#if HAL_UART_STATS_ENABLED
void Empty::UARTDriver::uart_info(ExpandingString &str)
{
    str.printf("EMPTY\n");
}
#endif
