/*
  implement generic UARTDriver code, including port locking
 */
#include "AP_HAL.h"

void AP_HAL::UARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (lock_write_key != 0) {
        // silently fail
        return;
    }
    return _begin(baud, rxSpace, txSpace);
}

void AP_HAL::UARTDriver::begin(uint32_t baud)
{
    return begin(baud, 0, 0);
}

/*
  lock the uart for exclusive use by write_locked() and read_locked() with the right key
 */
bool AP_HAL::UARTDriver::lock_port(uint32_t write_key, uint32_t read_key)
{
    if (lock_write_key != 0 && write_key != lock_write_key && write_key != 0) {
        // someone else is using it
        return false;
    }
    if (lock_read_key != 0 && read_key != lock_read_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    lock_write_key = write_key;
    lock_read_key = read_key;
    return true;
}

void AP_HAL::UARTDriver::begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        // silently fail
        return;
    }
    return _begin(baud, rxSpace, txSpace);
}

/*
   write to a locked port. If port is locked and key is not correct then 0 is returned
   and write is discarded. All writes are non-blocking
*/
size_t AP_HAL::UARTDriver::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return 0;
    }
    return _write(buffer, size);
}

/*
   read from a locked port. If port is locked and key is not correct then -1 is returned
*/
ssize_t AP_HAL::UARTDriver::read_locked(uint8_t *buf, size_t count, uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return 0;
    }
    ssize_t ret = _read(buf, count);
#if AP_UART_MONITOR_ENABLED
    auto monitor = _monitor_read_buffer;
    if (monitor != nullptr && ret > 0) {
        monitor->write(buf, ret);
    }
#endif
    return ret;
}

uint32_t AP_HAL::UARTDriver::available_locked(uint32_t key)
{
    if (lock_read_key != 0 && lock_read_key != key) {
        return 0;
    }
    return _available();
}

size_t AP_HAL::UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (lock_write_key != 0) {
        return 0;
    }
    return _write(buffer, size);
}

size_t AP_HAL::UARTDriver::write(uint8_t c)
{
    return write(&c, 1);
}

size_t AP_HAL::UARTDriver::write(const char *str)
{
    return write((const uint8_t *)str, strlen(str));
}

ssize_t AP_HAL::UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    return read_locked(buffer, count, 0);
}

bool AP_HAL::UARTDriver::read(uint8_t &b)
{
    ssize_t n = read(&b, 1);
    return n > 0;
}

int16_t AP_HAL::UARTDriver::read(void)
{
    uint8_t b;
    if (!read(b)) {
        return -1;
    }
    return b;
}


uint32_t AP_HAL::UARTDriver::available()
{
    if (lock_read_key != 0) {
        return 0;
    }
    return _available();
}

void AP_HAL::UARTDriver::end()
{
    if (lock_read_key != 0 || lock_write_key != 0) {
        return;
    }
    _end();
}

void AP_HAL::UARTDriver::flush()
{
    if (lock_read_key != 0 || lock_write_key != 0) {
        return;
    }
    _flush();
}

bool AP_HAL::UARTDriver::discard_input()
{
    if (lock_read_key != 0) {
        return false;
    }
    return _discard_input();
}
