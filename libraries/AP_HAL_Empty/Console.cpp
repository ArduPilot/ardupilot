#include <stdarg.h>
#include "Console.h"

using namespace Empty;

EmptyConsoleDriver::EmptyConsoleDriver(AP_HAL::BetterStream* delegate) :
    _d(delegate)
{}

void EmptyConsoleDriver::init(void* machtnichts)
{}

void EmptyConsoleDriver::backend_open()
{}

void EmptyConsoleDriver::backend_close()
{}

size_t EmptyConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t EmptyConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

int16_t EmptyConsoleDriver::available() {
    return _d->available();
}

int16_t EmptyConsoleDriver::txspace() {
    return _d->txspace();
}

int16_t EmptyConsoleDriver::read() {
    return _d->read();
}

size_t EmptyConsoleDriver::write(uint8_t c) {
    return _d->write(c);
}

