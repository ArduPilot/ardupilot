
#include "interface.h"
#include "UARTDriver.h"
#include <AP_Common/ExpandingString.h>

#if HAL_GCS_ENABLED
#include <AP_HAL/utility/packetise.h>
#endif

extern const AP_HAL::HAL& hal;
using namespace QURT;

/* QURT implementations of virtual methods */
void UARTDriver::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_initialised) {
        return;
    }

    /* we have enough memory to have a larger transmit buffer for
     * all ports. This means we don't get delays while waiting to
     * write GPS config packets
     */
    if (rxS < 4096) {
        rxS = 4096;
    }
    if (txS < 4096) {
        txS = 4096;
    }

    WITH_SEMAPHORE(_write_mutex);

    if (_writebuf.set_size(txS) && _readbuf.set_size(rxS)) {
        _initialised = true;
    }
}

void UARTDriver::_end()
{
}

void UARTDriver::_flush()
{
}

bool UARTDriver::is_initialized()
{
    return _initialised;
}

bool UARTDriver::tx_pending()
{
    return (_writebuf.available() > 0);
}

uint32_t UARTDriver::_available()
{
    if (!_initialised) {
        return 0;
    }

    WITH_SEMAPHORE(_read_mutex);

    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_initialised) {
        return 0;
    }
    return _writebuf.space();
}

bool UARTDriver::_discard_input()
{
    if (!_initialised) {
        return false;
    }

    WITH_SEMAPHORE(_read_mutex);

    _readbuf.clear();
    return true;
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialised) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);

    return _writebuf.write(buffer, size);
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    if (!_initialised) {
        return 0;
    }

    WITH_SEMAPHORE(_read_mutex);

    return _readbuf.read(buffer, size);
}

/*
  push any pending bytes to/from the serial port. This is called at
  1kHz in the timer thread. Doing it this way reduces the system call
  overhead in the main task enormously.
 */
void UARTDriver::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }

    for (auto i=0; i<10; i++) {
        if (!_write_pending_bytes()) {
            break;
        }
    }

    _fill_read_buffer();
}

/*
  methods for UARTDriver_Console
 */
void UARTDriver_Console::printf(const char *fmt, ...)
{
    va_list ap;
    char buf[300];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAP_PRINTF(buf);
}


/*
  methods for UARTDriver_MAVLinkUDP
 */
typedef void (*mavlink_data_callback_t)(const struct qurt_rpc_msg *msg, void* p);
extern void register_mavlink_data_callback(uint8_t instance, mavlink_data_callback_t func, void *p);

UARTDriver_MAVLinkUDP::UARTDriver_MAVLinkUDP(uint8_t instance) : inst(instance)
{
    register_mavlink_data_callback(instance, _mavlink_data_cb, (void *) this);
}

void UARTDriver_MAVLinkUDP::check_rx_seq(uint32_t seq)
{
	if (seq != rx_seq)
	{
		HAP_PRINTF("Sequence mismatch for instance %u. Expected %u, got %u", inst, rx_seq, seq);
	}
	rx_seq++;
}

void UARTDriver_MAVLinkUDP::_mavlink_data_cb(const struct qurt_rpc_msg *msg, void *p)
{
    auto *driver = (UARTDriver_MAVLinkUDP *)p;
    driver->check_rx_seq(msg->seq);
    driver->_readbuf.write(msg->data, msg->data_length);
}

/*
  try to push out one lump of pending bytes
  return true if progress is made
 */
bool UARTDriver_MAVLinkUDP::_write_pending_bytes(void)
{
    WITH_SEMAPHORE(_write_mutex);

    // write any pending bytes
    const uint32_t available_bytes = _writebuf.available();
    uint16_t n = available_bytes;

    if (n > 0) {
        // send on MAVLink packet boundaries if possible
        n = mavlink_packetise(_writebuf, n);
    }

    if (n <= 0) {
        return false;
    }

    struct qurt_rpc_msg msg;
    if (n > sizeof(msg.data)) {
        return false;
    }
    msg.msg_id = QURT_MSG_ID_MAVLINK_MSG;
    msg.inst = inst;
    msg.seq = tx_seq++;
    msg.data_length = _writebuf.read(msg.data, n);

    return qurt_rpc_send(msg);
}

/*
  setup baudrate for this local UART
 */
void UARTDriver_Local::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (b == 0) {
        // re-open not needed
        return;
    }

    // QURT wants 420000 for CRSF, ArduPilot driver wants 416666
    if (b == 416666) {
        b = 420000;
    }

    UARTDriver::_begin(b, rxS, txS);

    if (baudrate != b || fd == -1) {
        int fd2 = sl_client_config_uart(port_id, b);
        if (fd2 == -1 && fd != -1) {
            // baudrate rejected, revert to last baudrate
            sl_client_config_uart(port_id, baudrate);
        }
        if (fd2 != -1) {
            baudrate = b;
            fd = fd2;
        }
    }
}

/*
  push out pending bytes
 */
bool UARTDriver_Local::_write_pending_bytes(void)
{
    WITH_SEMAPHORE(_write_mutex);
    if (fd == -1) {
        return false;
    }
    uint32_t available;
    const uint8_t *ptr = _writebuf.readptr(available);
    if (ptr != nullptr) {
        auto n = sl_client_uart_write(fd, (const char *)ptr, available);
        if (n > 0) {
            _writebuf.advance(n);
            return true;
        }
    }
    return false;
}

/*
  read from the UART into _readbuf
 */
void UARTDriver_Local::_fill_read_buffer(void)
{
    WITH_SEMAPHORE(_read_mutex);
    if (fd == -1) {
        return;
    }
    uint32_t n = _readbuf.space();
    if (n > 512) {
        n = 512;
    }
    char buf[n];
    auto nread = sl_client_uart_read(fd, buf, sizeof(buf));
    if (nread > 0) {
        _readbuf.write((const uint8_t *)buf, nread);
        receive_timestamp_us = AP_HAL::micros64();
    }
}

/*
  return timestamp estimate in microseconds for when the start of a
  nbytes packet arrived on the uart.
*/
uint64_t UARTDriver_Local::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = receive_timestamp_us;
    if (baudrate > 0) {
        // assume 10 bits per byte
        uint32_t transport_time_us = (1000000UL * 10UL / baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}
