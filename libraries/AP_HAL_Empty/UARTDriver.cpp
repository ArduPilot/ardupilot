
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

Empty::UARTDriver::UARTDriver() {}

/* Empty implementations of virtual methods */
void Empty::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void Empty::UARTDriver::_end() {}
void Empty::UARTDriver::_flush() {}
bool Empty::UARTDriver::is_initialized() { return false; }
bool Empty::UARTDriver::tx_pending() { return false; }

uint32_t Empty::UARTDriver::_available() { return 0; }
uint32_t Empty::UARTDriver::txspace() { return 1; }
bool Empty::UARTDriver::_discard_input() { return false; }
size_t Empty::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return size;
}
ssize_t Empty::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return 0;
}

#if HAL_UART_STATS_ENABLED
void Empty::UARTDriver::uart_info(ExpandingString &str)
{
    str.printf("EMPTY\n");
}
#endif
