/*
  implement generic UARTDriver code, including port locking
 */
#include "AP_HAL.h"
#include <AP_Logger/AP_Logger.h>

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

/*
  default implementation of receive_time_constraint_us() will be used
  for subclasses that don't implement the call (eg. network
  sockets). Best we can do is to use the current timestamp as we don't
  know the transport delay
 */
uint64_t AP_HAL::UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    return AP_HAL::micros64();
}

#if HAL_UART_STATS_ENABLED
// Take cumulative bytes and return the change since last call
uint32_t AP_HAL::UARTDriver::StatsTracker::ByteTracker::update(uint32_t bytes)
{
    const uint32_t change = bytes - last_bytes;
    last_bytes = bytes;
    return change;
}

#if HAL_LOGGING_ENABLED
// Write UART log message
void AP_HAL::UARTDriver::log_stats(const uint8_t inst, StatsTracker &stats, const uint32_t dt_ms)
{
    // get totals
    const uint32_t total_tx_bytes = get_total_tx_bytes();
    const uint32_t total_rx_bytes = get_total_rx_bytes();

    // Don't log if we have never seen data
    if ((total_tx_bytes == 0) && (total_rx_bytes == 0)) {
        // This could be wrong if we happen to wrap both tx and rx to zero at exactly the same time
        // In that very unlikely case one log will be missed
        return;
    }

    // Update tracking
    const uint32_t tx_bytes = stats.tx.update(total_tx_bytes);
    const uint32_t rx_bytes = stats.rx.update(total_rx_bytes);

    // Assemble struct and log
    struct log_UART pkt {
        LOG_PACKET_HEADER_INIT(LOG_UART_MSG),
        time_us  : AP_HAL::micros64(),
        instance : inst,
        tx_rate  : float((tx_bytes * 1000) / dt_ms),
        rx_rate  : float((rx_bytes * 1000) / dt_ms),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif // HAL_LOGGING_ENABLED
#endif // HAL_UART_STATS_ENABLED
