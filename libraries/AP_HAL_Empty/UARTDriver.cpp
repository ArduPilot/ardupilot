
#include "UARTDriver.h"

using namespace Empty;

EmptyUARTDriver::EmptyUARTDriver() {}

void EmptyUARTDriver::begin(uint32_t b) {}
void EmptyUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void EmptyUARTDriver::end() {}
void EmptyUARTDriver::flush() {}
bool EmptyUARTDriver::is_initialized() { return false; }
void EmptyUARTDriver::set_blocking_writes(bool blocking) {}
bool EmptyUARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
int16_t EmptyUARTDriver::available() { return 0; }
int16_t EmptyUARTDriver::txspace() { return 1; }
int16_t EmptyUARTDriver::read() { return -1; }

/* Empty implementations of Print virtual methods */
size_t EmptyUARTDriver::write(uint8_t c) { return 0; }

size_t EmptyUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
