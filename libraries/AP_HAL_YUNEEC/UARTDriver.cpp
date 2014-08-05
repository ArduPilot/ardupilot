#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "UARTDriver.h"

using namespace YUNEEC;

YUNEECUARTDriver::YUNEECUARTDriver() {}

void YUNEECUARTDriver::begin(uint32_t b) {}
void YUNEECUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void YUNEECUARTDriver::end() {}
void YUNEECUARTDriver::flush() {}
bool YUNEECUARTDriver::is_initialized() { return false; }
void YUNEECUARTDriver::set_blocking_writes(bool blocking) {}
bool YUNEECUARTDriver::tx_pending() { return false; }

/* YUNEEC implementations of Stream virtual methods */
int16_t YUNEECUARTDriver::available() { return 0; }
int16_t YUNEECUARTDriver::txspace() { return 1; }
int16_t YUNEECUARTDriver::read() { return -1; }

/* YUNEEC implementations of Print virtual methods */
size_t YUNEECUARTDriver::write(uint8_t c) { return 0; }

size_t YUNEECUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif
