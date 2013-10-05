#include <stdarg.h>
#include "Console.h"

using namespace Linux;

LinuxConsoleDriver::LinuxConsoleDriver(AP_HAL::BetterStream* delegate) :
    _d(delegate)
{}

void LinuxConsoleDriver::init(void* machtnichts)
{}

void LinuxConsoleDriver::backend_open()
{}

void LinuxConsoleDriver::backend_close()
{}

size_t LinuxConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t LinuxConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

int16_t LinuxConsoleDriver::available() {
    return _d->available();
}

int16_t LinuxConsoleDriver::txspace() {
    return _d->txspace();
}

int16_t LinuxConsoleDriver::read() {
    return _d->read();
}

size_t LinuxConsoleDriver::write(uint8_t c) 
{
        return _d->write(c);
}

size_t LinuxConsoleDriver::write(const uint8_t *buffer, size_t size) {
    return _d->write(buffer, size);
}


