
#include "UARTDriver.h"

using namespace Empty;

UARTDriver::UARTDriver() {}

void UARTDriver::begin(uint32_t b) {}
void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void UARTDriver::end() {}
void UARTDriver::flush() {}
bool UARTDriver::is_initialized() { return false; }
void UARTDriver::set_blocking_writes(bool blocking) {}
bool UARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
int16_t UARTDriver::available() { return 0; }
int16_t UARTDriver::txspace() { return 1; }
int16_t UARTDriver::read() { return -1; }

/* Empty implementations of Print virtual methods */
size_t UARTDriver::write(uint8_t c) { return 0; }

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
