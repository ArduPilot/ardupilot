/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Siddharth Bharat Purohit
 * Referenced from implementation by Pavel Kirienko <pavel.kirienko@zubax.com>
 * for Zubax Babel
 */

#include <AP_HAL/AP_HAL.h>

#if AP_UAVCAN_SLCAN_ENABLED

#include "AP_UAVCAN_SLCAN.h"
#include <AP_SerialManager/AP_SerialManager.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CANSerialRouter.h>
#endif
#include <AP_Filesystem/AP_Filesystem.h>

extern const AP_HAL::HAL& hal;

static uint8_t nibble2hex(uint8_t x)
{
    // Allocating in RAM because it's faster
    static uint8_t ConversionTable[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}

static bool hex2nibble_error;

static uint8_t hex2nibble(char c)
{
    // Must go into RAM, not flash, because flash is slow
    static uint8_t NumConversionTable[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    };

    static uint8_t AlphaConversionTable[] = {
        10, 11, 12, 13, 14, 15
    };

    uint8_t out = 255;

    if (c >= '0' && c <= '9') {
        out = NumConversionTable[int(c) - int('0')];
    }
    else if (c >= 'a' && c <= 'f') {
        out = AlphaConversionTable[int(c) - int('a')];
    }
    else if (c >= 'A' && c <= 'F') {
        out = AlphaConversionTable[int(c) - int('A')];
    }

    if (out == 255) {
        hex2nibble_error = true;
    }
    return out;
}


bool SLCAN::CAN::push_Frame(uavcan::CanFrame &frame)
{
    SLCAN::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.utc_usec = AP_HAL::micros64();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    ChibiOS_CAN::CanIface::slcan_router().route_frame_to_can(frm.frame, frm.utc_usec);
#endif
    return rx_queue_.push(frm);
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CAN::handle_FrameDataExt(const char* cmd)
{
    uavcan::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + uavcan::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';
    if (f.dlc > uavcan::CanFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CAN::handle_FrameDataStd(const char* cmd)
{
    uavcan::CanFrame f;
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + uavcan::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc > uavcan::CanFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CAN::handle_FrameRTRExt(const char* cmd)
{
    uavcan::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + uavcan::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';

    if (f.dlc > uavcan::CanFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CAN::handle_FrameRTRStd(const char* cmd)
{
    uavcan::CanFrame f;
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + uavcan::CanFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc <= uavcan::CanFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

static inline const char* getASCIIStatusCode(bool status)
{
    return status ? "\r" : "\a";
}


bool SLCAN::CANManager::begin(uint32_t bitrate, uint8_t can_number)
{
    AP_HAL::UARTDriver *ser_port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SLCAN, can_number);
    if (driver_[can_number]->init(bitrate, SLCAN::CAN::NormalMode, ser_port) < 0) {
        return false;
    }
    if (initialized_) {
        return true;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCAN::CANManager::reader_trampoline, void), "SLCAN", 4096, AP_HAL::Scheduler::PRIORITY_CAN, -1)) {
        return false;
    }
    initialized(true);
    return true;
}

bool SLCAN::CANManager::is_initialized()
{
    return initialized_;
}

void SLCAN::CANManager::initialized(bool val)
{
    initialized_ = val;
}

int SLCAN::CAN::init(const uint32_t bitrate, const OperatingMode mode, AP_HAL::UARTDriver* port)
{
    if (port == nullptr) {
        return -1;
    }
    _port = port;
    initialized_ = true;
    return 0;
}

int SLCAN::CAN::init(const uint32_t bitrate, const OperatingMode mode, int __read_fd, int __write_fd)
{
    if (__read_fd == -1 || __write_fd == -1) {
        return -1;
    }
    _read_fd = __read_fd;
    _write_fd = __write_fd;
    initialized_ = true;
    return 0;
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data> [timestamp msec] [flags]
 * Types:
 *  R - RTR extended
 *  r - RTR standard
 *  T - Data extended
 *  t - Data standard
 * Flags:
 *  L - this frame is a loopback frame; timestamp field contains TX timestamp
 */
int16_t SLCAN::CAN::reportFrame(const uavcan::CanFrame& frame, bool loopback, uint64_t timestamp_usec)
{
    constexpr unsigned SLCANMaxFrameSize = 40;
    uint8_t buffer[SLCANMaxFrameSize] = {'\0'};
    uint8_t* p = &buffer[0];
    /*
        * Frame type
        */
    if (frame.isRemoteTransmissionRequest()) {
        *p++ = frame.isExtended() ? 'R' : 'r';
    }
    else if (frame.isErrorFrame()) {
        return -1;     // Not supported
    }
    else {
        *p++ = frame.isExtended() ? 'T' : 't';
    }

    /*
        * ID
        */
    {
        const uint32_t id = frame.id & frame.MaskExtID;
        if (frame.isExtended()) {
            *p++ = nibble2hex(id >> 28);
            *p++ = nibble2hex(id >> 24);
            *p++ = nibble2hex(id >> 20);
            *p++ = nibble2hex(id >> 16);
            *p++ = nibble2hex(id >> 12);
        }
        *p++ = nibble2hex(id >> 8);
        *p++ = nibble2hex(id >> 4);
        *p++ = nibble2hex(id >> 0);
    }

    /*
        * DLC
        */
    *p++ = char('0' + frame.dlc);

    /*
        * Data
        */
    for (unsigned i = 0; i < frame.dlc; i++) {
        const uint8_t byte = frame.data[i];
        *p++ = nibble2hex(byte >> 4);
        *p++ = nibble2hex(byte);
    }

    /*
        * Timestamp
        */
    //if (param_cache.timestamping_on)
    {
        // SLCAN format - [0, 60000) milliseconds
        const auto slcan_timestamp = uint16_t(timestamp_usec / 1000U);
        *p++ = nibble2hex(slcan_timestamp >> 12);
        *p++ = nibble2hex(slcan_timestamp >> 8);
        *p++ = nibble2hex(slcan_timestamp >> 4);
        *p++ = nibble2hex(slcan_timestamp >> 0);
    }

    /*
        * Flags
        */
    //if (param_cache.flags_on)
    {
        if (loopback) {
            *p++ = 'L';
        }
    }

    /*
        * Finalization
        */
    *p++ = '\r';
    const auto frame_size = unsigned(p - &buffer[0]);

    if (_port != nullptr ) {
        if (_port->txspace() < _pending_frame_size) {
            _pending_frame_size = frame_size;
            return 0;
        }
        //Write to Serial
        if (!_port->write_locked(&buffer[0], frame_size, _serial_lock_key)) {
            return 0;
        }
    } else if (_write_fd != -1) {
        int ret = write_fd(&buffer[0], frame_size);
        if (ret <= 0) {
            return 0;
        }
    }
    return 1;
}


/**
 * Accepts command string, returns response string or nullptr if no response is needed.
 */
const char* SLCAN::CAN::processCommand(char* cmd)
{
    /*
        * High-traffic SLCAN commands go first
        */
    if (cmd[0] == 'T') {
        return handle_FrameDataExt(cmd) ? "Z\r" : "\a";
    }
    else if (cmd[0] == 't') {
        return handle_FrameDataStd(cmd) ? "z\r" : "\a";
    }
    else if (cmd[0] == 'R') {
        return handle_FrameRTRExt(cmd) ? "Z\r" : "\a";
    }
    else if (cmd[0] == 'r' && cmd[1] <= '9') { // The second condition is needed to avoid greedy matching
        // See long commands below
        return handle_FrameRTRStd(cmd) ? "z\r" : "\a";
    }

    /*
        * Regular SLCAN commands
        */
    switch (cmd[0]) {
    case 'S':               // Set CAN bitrate
    case 'O':               // Open CAN in normal mode
    case 'L':               // Open CAN in listen-only mode
    case 'l': {             // Open CAN with loopback enabled
        _close = false;
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'C': {             // Close CAN
        _close = true;
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'M':               // Set CAN acceptance filter ID
    case 'm':               // Set CAN acceptance filter mask
    case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
    case 'Z': {             // Enable/disable RX and loopback timestamping
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'F': {             // Get status flags
        if (_port != nullptr) {
            _port->printf("F%02X\r", unsigned(0));    // Returning success for compatibility reasons
        } else if (_write_fd != -1) {
            char* data;
            if (asprintf(&data, "F%02X\r", unsigned(0)) > 0) {
                write_fd((const uint8_t*)data, sizeof(data));
                free(data);
            }
        }
        return nullptr;
    }
    case 'V': {             // HW/SW version
        if (_port != nullptr) {
            _port->printf("V%x%x%x%x\r", AP_UAVCAN_HW_VERS_MAJOR, AP_UAVCAN_HW_VERS_MINOR, AP_UAVCAN_SW_VERS_MAJOR, AP_UAVCAN_SW_VERS_MINOR);
        } else if (_write_fd != -1) {
            char* data;
            if (asprintf(&data, "V%x%x%x%x\r", AP_UAVCAN_HW_VERS_MAJOR, AP_UAVCAN_HW_VERS_MINOR, AP_UAVCAN_SW_VERS_MAJOR, AP_UAVCAN_SW_VERS_MINOR) > 0) {
                write_fd((const uint8_t*)data, sizeof(data));
                free(data);
            }
        }
        return nullptr;
    }
    case 'N': {             // Serial number
        uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion
        const uint8_t uid_buf_len = hw_version.unique_id.capacity();
        uint8_t uid_len = uid_buf_len;
        uint8_t *unique_id = new uint8_t[uid_buf_len];
        if (unique_id == nullptr) {
            break;
        } 
        char *buf = new char[uid_buf_len * 2 + 1];
        if (buf == nullptr) {
            break;
        } 
        memset(buf, 0, uid_buf_len * 2 + 1);
        char* pos = &buf[0];
        if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
            for (uint8_t i = 0; i < uid_buf_len; i++) {
                *pos++ = nibble2hex(unique_id[i] >> 4);
                *pos++ = nibble2hex(unique_id[i]);
            }
        }
        *pos++ = '\0';
        if (_port != nullptr) {
            _port->printf("N%s\r", &buf[0]);
        } else if (_write_fd != -1) {
            char* data;
            if (asprintf(&data, "N%s\r", &buf[0]) > 0) {
                write_fd((const uint8_t*)data, sizeof(data));
                free(data);
            }
        }
        free(unique_id);
        free(buf);
        return nullptr;
    }
    default: {
        break;
    }
    }

    return getASCIIStatusCode(false);
}

/**
 * Please keep in mind that this function is strongly optimized for speed.
 */
void SLCAN::CAN::addByte(const uint8_t byte)
{
    if ((byte >= 32 && byte <= 126)) {                // Normal printable ASCII character
        if (pos_ < SLCAN_BUFFER_SIZE) {
            buf_[pos_] = char(byte);
            pos_ += 1;
        }
        else {
            reset();                                        // Buffer overrun; silently drop the data
        }
    }
    else if (byte == '\r') {                          // End of command (SLCAN)
        // Processing the command
        buf_[pos_] = '\0';
        const char* const response = processCommand(reinterpret_cast<char*>(&buf_[0]));
        reset();

        // Sending the response if provided
        if (response != nullptr) {
            if (_port != nullptr) {
                _port->write_locked(reinterpret_cast<const uint8_t*>(response),
                                strlen(response), _serial_lock_key);
            } else if (_write_fd != -1) {
                write_fd(reinterpret_cast<const uint8_t*>(response),
                                strlen(response));
            }
        }
    }
    else if (byte == 8 || byte == 127) {            // DEL or BS (backspace)
        if (pos_ > 0) {
            pos_ -= 1;
        }
    }
    else {                                                  // This also includes Ctrl+C, Ctrl+D
        reset();                                            // Invalid byte - drop the current command
    }
}

void SLCAN::CAN::reset()
{
    pos_ = 0;
}



int SLCAN::CAN::read_fd(const uint8_t *buf, uint16_t count)
{
#ifdef FIONREAD
    // use FIONREAD to get exact value if possible
    int num_ready;
    while (ioctl(_read_fd, FIONREAD, &num_ready) == 0 && num_ready > 3000) {
        // the pipe is filling up - drain it
        uint8_t tmp[128];
        if (read(_read_fd, tmp, sizeof(tmp)) != sizeof(tmp)) {
            break;
        }
    }
#endif
    return AP::FS().read(_read_fd, (void*)buf, count);
}

int SLCAN::CAN::write_fd(const uint8_t *buf, uint16_t size)
{
    return (int)AP::FS().write(_write_fd, (const void*)buf, size);
}

void SLCAN::CAN::reader()
{
    if (_port != nullptr) {
        _port->lock_port(_serial_lock_key, _serial_lock_key);
        int16_t data = _port->read_locked(_serial_lock_key);
        while (data > 0) {
            addByte(data);
            data = _port->read_locked(_serial_lock_key);
        }
    } else if (_read_fd != -1) {
        uint8_t data = 0;
        while (true) {
            int ret = read_fd(&data, 1);
            if (ret == -1) {
                break;
            }
            addByte(data);
        }
    }
}

int16_t SLCAN::CAN::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -ErrUnsupportedFrame;
    }

    return reportFrame(frame, flags & uavcan::CanIOFlagLoopback, AP_HAL::micros64());
}

int16_t SLCAN::CAN::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                            uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());; // High precision is not required for monotonic timestamps
    uint64_t utc_usec;
    CanRxItem frm;
    rx_queue_.pop(frm);
    out_frame = frm.frame;
    utc_usec = frm.utc_usec;
    out_flags = frm.flags;
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

bool SLCAN::CAN::pending_frame_sent()
{
    if (_pending_frame_size == 0) {
        return false;
    } else if (_port != nullptr && _port->txspace() >= _pending_frame_size) {
        _pending_frame_size = 0;
        return true;
    } else if (_write_fd != -1) {
        return true;
    }
    return false;
}

bool SLCAN::CAN::isRxBufferEmpty()
{
    return rx_queue_.available() == 0;
}

bool SLCAN::CAN::canAcceptNewTxFrame() const
{
    constexpr unsigned SLCANMaxFrameSize = 40;
    if (_port != nullptr && _port->txspace() >= SLCANMaxFrameSize) {
        return true;
    } else if (_write_fd != -1) {
        return true;
    }
    return false;
}

uavcan::CanSelectMasks SLCAN::CANManager::makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces])
{
    uavcan::CanSelectMasks msk;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (!driver_[i]->is_initialized()) {
            continue;
        }

        if (!driver_[i]->isRxBufferEmpty()) {
            msk.read |= 1 << i;
        }

        if (pending_tx[i] != nullptr) {
            if (driver_[i]->canAcceptNewTxFrame()) {
                msk.write |= 1 << i;
            }
        }
    }

    return msk;
}

int16_t SLCAN::CANManager::select(uavcan::CanSelectMasks& inout_masks,
                                  const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());

    inout_masks = makeSelectMasks(pending_tx); // Check if we already have some of the requested events
    if ((inout_masks.read & in_masks.read) != 0 || (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    _irq_handler_ctx = chThdGetSelfX();
#else
    _irq_handler_ctx = pthread_self();
#endif
    if (blocking_deadline.toUSec()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I((blocking_deadline - time).toUSec())); // Block until timeout expires or any iface updates
#else
        struct timespec req;
#ifdef __APPLE__
        req.tv_sec =  (time_t)((blocking_deadline - time).toUSec()/1000000);
        req.tv_nsec = ((blocking_deadline - time).toUSec() % 1000000) * 1000;
        pthread_cond_timedwait_relative_np(&_irq_handler_cond, &_irq_handler_mtx, &req);
#else
        pthread_condattr_t attr;
        pthread_condattr_init( &attr);
        pthread_condattr_setclock( &attr, CLOCK_MONOTONIC);
        pthread_cond_init( &_irq_handler_cond, &attr);
        clock_gettime(CLOCK_MONOTONIC, &req);
        pthread_mutex_lock(&_irq_handler_mtx);
        req.tv_sec +=  (time_t)((blocking_deadline - time).toUSec()/1000000);
        req.tv_nsec += ((blocking_deadline - time).toUSec() % 1000000) * 1000;
        pthread_cond_timedwait(&_irq_handler_cond, &_irq_handler_mtx, &req);
#endif
        pthread_mutex_unlock(&_irq_handler_mtx);
#endif
    }
    inout_masks = makeSelectMasks(pending_tx); // Return what we got even if none of the requested events are set
    return 1; // Return value doesn't matter as long as it is non-negative
}

void SLCAN::CANManager::reader_trampoline(void)
{
    while (true) {
        driver_[0]->reader();
        driver_[1]->reader();
        if ((driver_[0]->pending_frame_sent() || !driver_[0]->isRxBufferEmpty() ||
             driver_[1]->pending_frame_sent() || !driver_[1]->isRxBufferEmpty()) && 
             _irq_handler_ctx) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
            chEvtSignalI(_irq_handler_ctx, EVENT_MASK(0));
#else
            pthread_mutex_lock(&_irq_handler_mtx);
            pthread_cond_signal(&_irq_handler_cond);
            pthread_mutex_unlock(&_irq_handler_mtx);
#endif
        }
        hal.scheduler->delay_microseconds(100);
    }
}

#endif // AP_UAVCAN_SLCAN_ENABLED

