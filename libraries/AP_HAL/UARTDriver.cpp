/*
  implement generic UARTDriver code, including port locking
 */
#include "AP_HAL.h"
#include <AP_Logger/AP_Logger.h>

#if HAL_UART_DEBUG_LOGGING_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#endif // HAL_UART_DEBUG_LOGGING_ENABLED

extern const AP_HAL::HAL& hal;

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
    const size_t ret = _write(buffer, size);
#if HAL_UART_DEBUG_LOGGING_ENABLED
    if (ret > 0) {
        log_written_data(buffer, ret);
    }
#endif
    return ret;
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
#if HAL_UART_DEBUG_LOGGING_ENABLED
    if (ret > 0) {
        log_read_data(buf, ret);
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
    const size_t ret = _write(buffer, size);
#if HAL_UART_DEBUG_LOGGING_ENABLED
    log_written_data(buffer, ret);
#endif
    return ret;
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

// Helper to check if flow control is enabled given the passed setting
bool AP_HAL::UARTDriver::flow_control_enabled(enum flow_control flow_control_setting) const
{
    switch(flow_control_setting) {
        case FLOW_CONTROL_ENABLE:
        case FLOW_CONTROL_AUTO:
            return true;
        case FLOW_CONTROL_DISABLE:
        case FLOW_CONTROL_RTS_DE:
            break;
    }
    return false;
}

uint8_t AP_HAL::UARTDriver::get_parity(void)
{
    return AP_HAL::UARTDriver::parity;
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
    const uint32_t rx_dropped_bytes = stats.rx_dropped.update(get_total_dropped_rx_bytes());

    // Assemble struct and log
    struct log_UART pkt {
        LOG_PACKET_HEADER_INIT(LOG_UART_MSG),
        time_us  : AP_HAL::micros64(),
        instance : inst,
        tx_rate  : float((tx_bytes * 1000) / dt_ms),
        rx_rate  : float((rx_bytes * 1000) / dt_ms),
        rx_drop_rate : float((rx_dropped_bytes * 1000) / dt_ms),
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif // HAL_LOGGING_ENABLED
#endif // HAL_UART_STATS_ENABLED


/*
 * Support for logging all incoming/outgoing data from a serial port
 */

#if HAL_UART_DEBUG_LOGGING_ENABLED

/*
  log some data for debugging
*/
void AP_HAL::UARTDriver::log_data(const uint8_t *data, uint16_t length, bool is_written_data)
{
    // first see if we actually do want to log:
    if ((_last_options & OPTION_LOGGING) == 0) {
        return;
    }

    // find logging instance corresponding to this UART:
    if (loginfo == nullptr) {
        WITH_SEMAPHORE(loginfo_creation_sem);

        loginfo = NEW_NOTHROW LogInfo();
        if (loginfo == nullptr) {
            return;
        }

        loginfo->instance = instance;
        loginfo->next = backend_loginfos;
        backend_loginfos = loginfo;

        if (!log_thread_created) {
            log_thread_created = true;
            hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_HAL::UARTDriver::logging_start, void), "uart_log", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0);
        }
    }

    // write packet into buffer:
    struct PACKED Header {
        enum class Flag {
            IS_WRITTEN_DATA = 1,
        };
        uint32_t magic = 0xEAEF0D0F;
        uint32_t time_ms;
        uint16_t length;
        uint8_t flags;
    };

    // note that we are assuming we can do this writing atomically.
    // If there are multiple writers to the serial port that won't be
    // the case.  We could reserve, write to and commit an iovec
    // instead of this:

    const auto space_required = sizeof(Header) + length;

    if (loginfo->buf.space() < space_required) {
        return;
    }

    Header header;
    header.time_ms = AP_HAL::millis();
    header.length = length;
    if (is_written_data) {
        header.flags |= uint8_t(Header::Flag::IS_WRITTEN_DATA);
    }

    loginfo->buf.write((uint8_t*)&header, sizeof(header));
    loginfo->buf.write(data, length);
}

// a linked list of backends which are currently logging:
AP_HAL::UARTDriver::LogInfo *AP_HAL::UARTDriver::backend_loginfos;
bool AP_HAL::UARTDriver::log_thread_created;

// logging loop, needs to be static as it loops over backends created
// by all all backends
void AP_HAL::UARTDriver::logging_loop(void)
{
    while (true) {
        hal.scheduler->delay(10);
        for (auto *loginfo_ptr = backend_loginfos; loginfo_ptr != nullptr; loginfo_ptr = loginfo_ptr->next) {
            auto &loginfo = *loginfo_ptr;
            if (loginfo.fd == -2) {
                // previously failed; ignore.
                continue;
            }
            // open logfile if required:
            if (loginfo.fd == -1) {
                char fname[20];
                for (uint32_t lognum=0; lognum<1000; lognum++) {
                    hal.util->snprintf(fname, ARRAY_SIZE(fname), "uart%u_%03u.log", loginfo.instance, (unsigned)lognum);
                    struct stat st;
                    if (AP::FS().stat(fname, &st) != 0) {
                        break;
                    }
                }
                loginfo.fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_APPEND);
                if (loginfo.fd == -1) {
                    // convert into a permanent failure:
                    loginfo.fd = -2;
                    continue;
                }
            }
            uint32_t n = 0;
            const uint8_t *p;
            while ((p = loginfo.buf.readptr(n)) != nullptr && n != 0) {
                // short writes are unlikely and are ignored (only FS full errors)
                AP::FS().write(loginfo.fd, p, n);
                loginfo.buf.advance(n);
            }
            AP::FS().fsync(loginfo.fd);
        }
    }
}

// logging thread start, needs to be non-static for thread_create
void AP_HAL::UARTDriver::logging_start(void)
{
    logging_loop();
}
#endif // HAL_UART_DEBUG_LOGGING_ENABLED
