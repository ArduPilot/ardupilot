
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

Embox::UARTDriver::UARTDriver() {}

/* Embox implementations of virtual methods */
void Embox::UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void Embox::UARTDriver::_end() {}
void Embox::UARTDriver::_flush() {}
bool Embox::UARTDriver::is_initialized() { return false; }
bool Embox::UARTDriver::tx_pending() { return false; }

uint32_t Embox::UARTDriver::_available() { return 0; }
uint32_t Embox::UARTDriver::txspace() { return 1; }
bool Embox::UARTDriver::_discard_input() { return false; }
size_t Embox::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return size;
}
ssize_t Embox::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return 0;
}

#if HAL_UART_STATS_ENABLED
void Embox::UARTDriver::uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms)
{
    str.printf("EMPTY\n");
}
#endif
