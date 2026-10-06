
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
    qurt_printf_to_host(buf);
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

/*
  methods for UARTDriver_RemoteRegistered — a dynamic serial port that
  tunnels to a UART on the apps processor. The port is registered with
  AP_SerialManager at boot; the user selects a protocol for it via the
  SERIALn_PROTOCOL parameter matching the registered state.idx.
*/
typedef void (*remote_uart_data_callback_t)(const struct qurt_rpc_msg *msg, void *p);
extern void register_remote_uart_data_callback(uint8_t port_id,
                                               remote_uart_data_callback_t func,
                                               void *p);

void UARTDriver_RemoteRegistered::init(uint8_t serial_idx)
{
    state.idx = serial_idx;
    // this port's protocol and baud come from the matching SERIALn_*
    // parameters via AP_SerialManager::state[serial_idx]; leave our own
    // state.protocol at SerialProtocol_None so it is not matched in the
    // registered_ports search path of find_protocol_instance()
    state.protocol.set(AP_SerialManager::SerialProtocol_None);
    // register once; data callbacks start flowing immediately
    register_remote_uart_data_callback(port_id, uart_data_cb, (void *)this);
    AP::serialmanager().register_port(this);
}

uint32_t UARTDriver_RemoteRegistered::get_baud_rate() const
{
    // once _begin() has been called, return the negotiated baud
    if (baudrate != 0) {
        return baudrate;
    }
    // before first _begin(), fall back to the SERIALn_BAUD configured
    // for our slot. AP_GPS's auto-baud detection calls get_baud_rate()
    // to seed its first probe; returning 0 here would wedge the probe
    // in a begin(0) loop (_begin skips send_config when b==0).
    const auto *st = AP::serialmanager().get_state_by_id(state.idx);
    return st != nullptr ? st->baudrate() : 0;
}

bool UARTDriver_RemoteRegistered::tx_pending()
{
    WITH_SEMAPHORE(write_mutex);
    return writebuffer != nullptr && writebuffer->available() > 0;
}

uint32_t UARTDriver_RemoteRegistered::txspace()
{
    WITH_SEMAPHORE(write_mutex);
    return writebuffer != nullptr ? writebuffer->space() : 0;
}

bool UARTDriver_RemoteRegistered::send_config(uint32_t b)
{
    struct qurt_rpc_msg msg {};
    msg.msg_id = QURT_MSG_ID_UART_CONFIG;
    msg.inst = port_id;
    msg.seq = 0;
    struct qurt_uart_config cfg {};
    cfg.baudrate = b;
    cfg.device_id = device_id;
    cfg.port_id = port_id;
    msg.data_length = sizeof(cfg);
    memcpy(msg.data, &cfg, sizeof(cfg));
    if (!qurt_rpc_send(msg)) {
        DEV_PRINTF("Remote UART %u: config send failed (baud=%lu device=%lu)",
                   (unsigned)port_id,
                   (unsigned long)b,
                   (unsigned long)device_id);
        return false;
    }
    return true;
}

void UARTDriver_RemoteRegistered::_begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (rxS < 4096) {
        rxS = 4096;
    }
    if (txS < 4096) {
        txS = 4096;
    }

    {
        WITH_SEMAPHORE(read_mutex);
        if (readbuffer == nullptr) {
            readbuffer = NEW_NOTHROW ByteBuffer(rxS);
        } else {
            readbuffer->set_size_best(rxS);
        }
    }
    {
        WITH_SEMAPHORE(write_mutex);
        if (writebuffer == nullptr) {
            writebuffer = NEW_NOTHROW ByteBuffer(txS);
        } else {
            writebuffer->set_size_best(txS);
        }
    }

    initialised = (readbuffer != nullptr) && (writebuffer != nullptr);

    if (b != 0 && initialised && (!remote_configured || baudrate != b)) {
        {
            WITH_SEMAPHORE(read_mutex);
            readbuffer->clear();
        }
        if (send_config(b)) {
            baudrate = b;
            remote_configured = true;
        }
    }
}

size_t UARTDriver_RemoteRegistered::_write(const uint8_t *buffer, size_t size)
{
    WITH_SEMAPHORE(write_mutex);
    return writebuffer != nullptr ? writebuffer->write(buffer, size) : 0;
}

ssize_t UARTDriver_RemoteRegistered::_read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(read_mutex);
    return readbuffer != nullptr ? readbuffer->read(buffer, count) : -1;
}

uint32_t UARTDriver_RemoteRegistered::_available()
{
    WITH_SEMAPHORE(read_mutex);
    return readbuffer != nullptr ? readbuffer->available() : 0;
}

bool UARTDriver_RemoteRegistered::_discard_input()
{
    WITH_SEMAPHORE(read_mutex);
    if (readbuffer != nullptr) {
        readbuffer->clear();
    }
    return true;
}

bool UARTDriver_RemoteRegistered::push_pending_bytes()
{
    if (!remote_configured) {
        return false;
    }

    WITH_SEMAPHORE(write_mutex);

    if (writebuffer == nullptr) {
        return false;
    }

    uint32_t available;
    const uint8_t *ptr = writebuffer->readptr(available);
    if (ptr == nullptr || available == 0) {
        return false;
    }

    struct qurt_rpc_msg msg {};
    const uint16_t n = (uint16_t)(available < sizeof(msg.data) ? available : sizeof(msg.data));

    msg.msg_id = QURT_MSG_ID_UART_DATA;
    msg.inst = port_id;
    msg.seq = tx_seq++;
    msg.data_length = n;
    memcpy(msg.data, ptr, n);

    if (qurt_rpc_send(msg)) {
        writebuffer->advance(n);
        return true;
    }
    return false;
}

void UARTDriver_RemoteRegistered::_timer_tick(void)
{
    if (!initialised) {
        return;
    }
    // cap chunks per tick so one port can't starve the others in the
    // shared uart thread (each chunk is up to sizeof(qurt_rpc_msg::data))
    constexpr uint8_t MAX_TX_BURST = 10;
    for (uint8_t i = 0; i < MAX_TX_BURST; i++) {
        if (!push_pending_bytes()) {
            break;
        }
    }
}

void UARTDriver_RemoteRegistered::uart_data_cb(const struct qurt_rpc_msg *msg, void *p)
{
    auto *driver = (UARTDriver_RemoteRegistered *)p;
    if (msg->seq != driver->rx_seq) {
        DEV_PRINTF("Remote UART %u: RX seq mismatch expected=%lu got=%lu len=%u",
                   (unsigned)driver->port_id,
                   (unsigned long)driver->rx_seq,
                   (unsigned long)msg->seq,
                   msg->data_length);
        driver->rx_seq = msg->seq;
    }
    WITH_SEMAPHORE(driver->read_mutex);
    if (driver->readbuffer == nullptr) {
        return;
    }
    const uint16_t written = driver->readbuffer->write(msg->data, msg->data_length);
    if (written != msg->data_length) {
        DEV_PRINTF("Remote UART %u: RX buffer dropped %u/%u bytes (space=%u)",
                   (unsigned)driver->port_id,
                   (unsigned)(msg->data_length - written),
                   (unsigned)msg->data_length,
                   (unsigned)driver->readbuffer->space());
    }

    driver->rx_seq++;
}
