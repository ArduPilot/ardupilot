/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Many thanks to members of the UAVCAN project:
 *  Pavel Kirienko <pavel.kirienko@gmail.com>
 *  Ilia Sheremet <illia.sheremet@gmail.com>
 *
 *  license info can be found in the uavcan submodule located:
 *  modules/uavcan/libuavcan_drivers/linux/include/uavcan_linux/socketcan.hpp
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#if HAL_NUM_CAN_IFACES

#include "CANSocketIface.h"

#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <cstring>
#include "Scheduler.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANLinuxIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

CANIface::CANSocketEventSource CANIface::evt_can_socket[HAL_NUM_CAN_IFACES];

uint8_t CANIface::next_interface;

static can_frame makeSocketCanFrame(const AP_HAL::CANFrame& uavcan_frame)
{
    can_frame sockcan_frame { uavcan_frame.id& AP_HAL::CANFrame::MaskExtID, uavcan_frame.dlc, { } };
    std::copy(uavcan_frame.data, uavcan_frame.data + uavcan_frame.dlc, sockcan_frame.data);
    if (uavcan_frame.isExtended()) {
        sockcan_frame.can_id |= CAN_EFF_FLAG;
    }
    if (uavcan_frame.isErrorFrame()) {
        sockcan_frame.can_id |= CAN_ERR_FLAG;
    }
    if (uavcan_frame.isRemoteTransmissionRequest()) {
        sockcan_frame.can_id |= CAN_RTR_FLAG;
    }
    return sockcan_frame;
}

static canfd_frame makeSocketCanFDFrame(const AP_HAL::CANFrame& uavcan_frame)
{
    canfd_frame sockcan_frame { uavcan_frame.id& AP_HAL::CANFrame::MaskExtID, AP_HAL::CANFrame::dlcToDataLength(uavcan_frame.dlc), CANFD_BRS, 0, 0, { } };
    std::copy(uavcan_frame.data, uavcan_frame.data + AP_HAL::CANFrame::dlcToDataLength(uavcan_frame.dlc), sockcan_frame.data);
    if (uavcan_frame.isExtended()) {
        sockcan_frame.can_id |= CAN_EFF_FLAG;
    }
    if (uavcan_frame.isErrorFrame()) {
        sockcan_frame.can_id |= CAN_ERR_FLAG;
    }
    if (uavcan_frame.isRemoteTransmissionRequest()) {
        sockcan_frame.can_id |= CAN_RTR_FLAG;
    }
    return sockcan_frame;
}

static AP_HAL::CANFrame makeCanFrame(const can_frame& sockcan_frame)
{
    AP_HAL::CANFrame can_frame(sockcan_frame.can_id & CAN_EFF_MASK, sockcan_frame.data, sockcan_frame.can_dlc);
    if (sockcan_frame.can_id & CAN_EFF_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagEFF;
    }
    if (sockcan_frame.can_id & CAN_ERR_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagERR;
    }
    if (sockcan_frame.can_id & CAN_RTR_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagRTR;
    }
    return can_frame;
}

static AP_HAL::CANFrame makeCanFDFrame(const canfd_frame& sockcan_frame)
{
    AP_HAL::CANFrame can_frame(sockcan_frame.can_id & CAN_EFF_MASK, sockcan_frame.data, sockcan_frame.len);
    if (sockcan_frame.can_id & CAN_EFF_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagEFF;
    }
    if (sockcan_frame.can_id & CAN_ERR_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagERR;
    }
    if (sockcan_frame.can_id & CAN_RTR_FLAG) {
        can_frame.id |= AP_HAL::CANFrame::FlagRTR;
    }
    return can_frame;
}

bool CANIface::is_initialized() const
{
    return _initialized;
}

int CANIface::_openSocket(const std::string& iface_name)
{
    errno = 0;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        return s;
    }

    std::shared_ptr<void> defer(&s, [](int* fd) { if (*fd >= 0) close(*fd); });
    const int ret = s;

    // Detect the iface index
    auto ifr = ifreq();
    if (iface_name.length() >= IFNAMSIZ) {
        errno = ENAMETOOLONG;
        return -1;
    }
    std::strncpy(ifr.ifr_name, iface_name.c_str(), iface_name.length());
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0 || ifr.ifr_ifindex < 0) {
        return -1;
    }

    // Bind to the specified CAN iface
    {
        auto addr = sockaddr_can();
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            return -1;
        }
    }

    // Configure
    {
        const int on = 1;
        // Timestamping
        if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
            return -1;
        }
        // Socket loopback
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &on, sizeof(on)) < 0) {
            return -1;
        }
        // Allow CANFD
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
            return -1;
        }
        // Non-blocking
        if (fcntl(s, F_SETFL, O_NONBLOCK) < 0) {
            return -1;
        }
    }

    // Validate the resulting socket
    {
        int socket_error = 0;
        socklen_t errlen = sizeof(socket_error);
        getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<void*>(&socket_error), &errlen);
        if (socket_error != 0) {
            errno = socket_error;
            return -1;
        }
    }
    s = -1;
    return ret;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, const uint64_t tx_deadline,
                       const CANIface::CanIOFlags flags)
{
    WITH_SEMAPHORE(sem);
    CanTxItem tx_item {};
    tx_item.frame = frame;
    if (flags & Loopback) {
        tx_item.loopback = true;
    }
    if (flags & AbortOnError) {
        tx_item.abort_on_error = true;
    }
    tx_item.setup = true;
    tx_item.index = _tx_frame_counter;
    tx_item.deadline = tx_deadline;
    _tx_queue.emplace(tx_item);
    _tx_frame_counter++;
    stats.tx_requests++;
    _pollRead();     // Read poll is necessary because it can release the pending TX flag
    _pollWrite();

    return AP_HAL::CANIface::send(frame, tx_deadline, flags);
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                          CANIface::CanIOFlags& out_flags)
{
    WITH_SEMAPHORE(sem);
    if (_rx_queue.empty()) {
        _pollRead();            // This allows to use the socket not calling poll() explicitly.
        if (_rx_queue.empty()) {
            return 0;
        }
    }
    {
        const CanRxItem& rx = _rx_queue.front();
        out_frame        = rx.frame;
        out_timestamp_us = rx.timestamp_us;
        out_flags        = rx.flags;
    }
    (void)_rx_queue.pop();
    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

bool CANIface::_hasReadyTx()
{
    WITH_SEMAPHORE(sem);
    return !_tx_queue.empty();
}

bool CANIface::_hasReadyRx()
{
    WITH_SEMAPHORE(sem);
    return !_rx_queue.empty();
}

void CANIface::_poll(bool read, bool write)
{
    if (read) {
        stats.num_poll_rx_events++;
        _pollRead(); // Read poll must be executed first because it may decrement _frames_in_socket_tx_queue
    }
    if (write) {
        stats.num_poll_tx_events++;
        _pollWrite();
    }
}

bool CANIface::configureFilters(const CanFilterConfig* const filter_configs,
                              const std::uint16_t num_configs)
{
#if 0
    if (filter_configs == nullptr || mode_ != FilteredMode) {
        return false;
    }
    _hw_filters_container.clear();
    _hw_filters_container.resize(num_configs);

    for (unsigned i = 0; i < num_configs; i++) {
        const CanFilterConfig& fc = filter_configs[i];
        _hw_filters_container[i].can_id   = fc.id   & AP_HAL::CANFrame::MaskExtID;
        _hw_filters_container[i].can_mask = fc.mask & AP_HAL::CANFrame::MaskExtID;
        if (fc.id & AP_HAL::CANFrame::FlagEFF) {
            _hw_filters_container[i].can_id |= CAN_EFF_FLAG;
        }
        if (fc.id & AP_HAL::CANFrame::FlagRTR) {
            _hw_filters_container[i].can_id |= CAN_RTR_FLAG;
        }
        if (fc.mask & AP_HAL::CANFrame::FlagEFF) {
            _hw_filters_container[i].can_mask |= CAN_EFF_FLAG;
        }
        if (fc.mask & AP_HAL::CANFrame::FlagRTR) {
            _hw_filters_container[i].can_mask |= CAN_RTR_FLAG;
        }
    }
#endif
    return true;
}

/**
 * SocketCAN emulates the CAN filters in software, so the number of filters is virtually unlimited.
 * This method returns a constant value.
 */
static constexpr unsigned NumFilters = CAN_FILTER_NUMBER;
uint16_t CANIface::getNumFilters() const { return NumFilters; }

uint32_t CANIface::getErrorCount() const
{
    uint32_t ec = 0;
    for (auto& kv : _errors) { ec += kv.second; }
    return ec;
}

void CANIface::_pollWrite()
{
    while (_hasReadyTx()) {
        WITH_SEMAPHORE(sem);
        const CanTxItem tx = _tx_queue.top();
        uint64_t curr_time = AP_HAL::native_micros64();
        if (tx.deadline >= curr_time) {
            // hal.console->printf("%x TDEAD: %lu CURRT: %lu DEL: %lu\n",tx.frame.id,  tx.deadline, curr_time, tx.deadline-curr_time);
            const int res = _write(tx.frame);
            if (res == 1) {                   // Transmitted successfully
                _incrementNumFramesInSocketTxQueue();
                if (tx.loopback) {
                    _pending_loopback_ids.insert(tx.frame.id);
                }
                stats.tx_success++;
            } else if (res == 0) {            // Not transmitted, nor is it an error
                stats.tx_full++;
                break;                        // Leaving the loop, the frame remains enqueued for the next retry
            } else {                          // Transmission error
                stats.tx_rejected++;
            }
        } else {
            // hal.console->printf("TDEAD: %lu CURRT: %lu DEL: %lu\n", tx.deadline, curr_time, curr_time-tx.deadline);
            stats.tx_timedout++;
        }

        // Removing the frame from the queue even if transmission failed
        (void)_tx_queue.pop();
    }
}

bool CANIface::_pollRead()
{
    uint8_t iterations_count = 0;
    while (iterations_count < CAN_MAX_POLL_ITERATIONS_COUNT)
    {
        iterations_count++;
        CanRxItem rx;
        rx.timestamp_us = AP_HAL::native_micros64();  // Monotonic timestamp is not required to be precise (unlike UTC)
        bool loopback = false;
        int res;
        if (iterations_count % 2 == 0) {
            res = _read(rx.frame, rx.timestamp_us, loopback);
        } else {
            res = _readfd(rx.frame, rx.timestamp_us, loopback);
        }
        if (res == 1) {
            bool accept = true;
            if (loopback) {           // We receive loopback for all CAN frames
                _confirmSentFrame();
                rx.flags |= Loopback;
                accept = _wasInPendingLoopbackSet(rx.frame);
                stats.tx_confirmed++;
            }
            if (accept) {
                WITH_SEMAPHORE(sem);
                add_to_rx_queue(rx);
                stats.rx_received++;
                return true;
            }
        } else if (res == 0) {
            break;
        } else {
            stats.rx_errors++;
            break;
        }
    }
    return false;
}

int CANIface::_write(const AP_HAL::CANFrame& frame) const
{
    if (_fd < 0) {
        return -1;
    }
    errno = 0;
    int res = 0;

    if (frame.isCanFDFrame()) {
        const canfd_frame sockcan_frame = makeSocketCanFDFrame(frame);
        res = write(_fd, &sockcan_frame, sizeof(sockcan_frame));
        if (res > 0 && res != sizeof(sockcan_frame)) {
            return -1;
        }
    } else {
        const can_frame sockcan_frame = makeSocketCanFrame(frame);
        res = write(_fd, &sockcan_frame, sizeof(sockcan_frame));
        if (res > 0 && res != sizeof(sockcan_frame)) {
            return -1;
        }
    }
    if (res <= 0) {
        if (errno == ENOBUFS || errno == EAGAIN) {  // Writing is not possible atm, not an error
            return 0;
        }
        return res;
    }
    return 1;
}


int CANIface::_read(AP_HAL::CANFrame& frame, uint64_t& timestamp_us, bool& loopback) const
{
    if (_fd < 0) {
        return -1;
    }
    auto iov = iovec();
    auto sockcan_frame = can_frame();
    iov.iov_base = &sockcan_frame;
    iov.iov_len  = sizeof(sockcan_frame);
    union {
        uint8_t data[CMSG_SPACE(sizeof(::timeval))];
        struct cmsghdr align;
    } control;

    auto msg = msghdr();
    msg.msg_iov    = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data;
    msg.msg_controllen = sizeof(control.data);

    const int res = recvmsg(_fd, &msg, MSG_DONTWAIT);
    if (res <= 0) {
        return (res < 0 && errno == EWOULDBLOCK) ? 0 : res;
    }
    /*
     * Flags
     */
    loopback = (msg.msg_flags & static_cast<int>(MSG_CONFIRM)) != 0;

    if (!loopback && !_checkHWFilters(sockcan_frame)) {
        return 0;
    }

    frame = makeCanFrame(sockcan_frame);
    /*
     * Timestamp
     */
    timestamp_us = AP_HAL::native_micros64();
    return 1;
}

int CANIface::_readfd(AP_HAL::CANFrame& frame, uint64_t& timestamp_us, bool& loopback) const
{
    if (_fd < 0) {
        return -1;
    }
    auto iov = iovec();
    auto sockcan_frame = canfd_frame();
    iov.iov_base = &sockcan_frame;
    iov.iov_len  = sizeof(sockcan_frame);
    union {
        uint8_t data[CMSG_SPACE(sizeof(::timeval))];
        struct cmsghdr align;
    } control;

    auto msg = msghdr();
    msg.msg_iov    = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = control.data;
    msg.msg_controllen = sizeof(control.data);

    const int res = recvmsg(_fd, &msg, MSG_DONTWAIT);
    if (res <= 0) {
        return (res < 0 && errno == EWOULDBLOCK) ? 0 : res;
    }
    /*
     * Flags
     */
    loopback = (msg.msg_flags & static_cast<int>(MSG_CONFIRM)) != 0;

    if (!loopback && !_checkHWFilters(sockcan_frame)) {
        return 0;
    }

    frame = makeCanFDFrame(sockcan_frame);
    /*
     * Timestamp
     */
    timestamp_us = AP_HAL::native_micros64();
    return 1;
}

// Might block forever, only to be used for testing
void CANIface::flush_tx()
{
    WITH_SEMAPHORE(sem);
    do {
        _updateDownStatusFromPollResult(_pollfd);
        _poll(true, true);
    } while(!_tx_queue.empty() && !_down);
}

void CANIface::clear_rx()
{
    WITH_SEMAPHORE(sem);
    // Clean Rx Queue
    std::queue<CanRxItem> empty;
    std::swap( _rx_queue, empty );
}

void CANIface::_incrementNumFramesInSocketTxQueue()
{
    _frames_in_socket_tx_queue++;
}

void CANIface::_confirmSentFrame()
{
    if (_frames_in_socket_tx_queue > 0) {
        _frames_in_socket_tx_queue--;
    }
}

bool CANIface::_wasInPendingLoopbackSet(const AP_HAL::CANFrame& frame)
{
    if (_pending_loopback_ids.count(frame.id) > 0) {
        _pending_loopback_ids.erase(frame.id);
        return true;
    }
    return false;
}

bool CANIface::_checkHWFilters(const can_frame& frame) const
{
    if (!_hw_filters_container.empty()) {
        for (auto& f : _hw_filters_container) {
            if (((frame.can_id & f.can_mask) ^ f.can_id) == 0) {
                return true;
            }
        }
        return false;
    } else {
        return true;
    }
}

bool CANIface::_checkHWFilters(const canfd_frame& frame) const
{
    if (!_hw_filters_container.empty()) {
        for (auto& f : _hw_filters_container) {
            if (((frame.can_id & f.can_mask) ^ f.can_id) == 0) {
                return true;
            }
        }
        return false;
    } else {
        return true;
    }
}

void CANIface::_updateDownStatusFromPollResult(const pollfd& pfd)
{
    if (!_down && (pfd.revents & POLLERR)) {
        int error = 0;
        socklen_t errlen = sizeof(error);
        getsockopt(pfd.fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<void*>(&error), &errlen);

        _down= error == ENETDOWN || error == ENODEV;
        stats.num_downs++;
        Debug("Iface %d is dead; error %d", _fd, error);
    }
}

bool CANIface::init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode)
{
    // we are using vcan, so bitrate is irrelevant
    return init(bitrate, mode);
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    char iface_name[16];
    sprintf(iface_name, "vcan%u", _self_index);
    bitrate_ = bitrate;
    mode_ = mode;
    if (_initialized) {
        return _initialized;
    }

    // TODO: Add possibility change bitrate
    _fd = _openSocket(iface_name);
    if (_fd > 0) {
        _bitrate = bitrate;
        _initialized = true;
    } else {
        _initialized = false;
    }
    return _initialized;
}

bool CANIface::select(bool &read_select, bool &write_select,
                        const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline)
{
    // Detecting whether we need to block at all
    bool need_block = !write_select;    // Write queue is infinite
    // call poll here to flush some tx
    _poll(true, true);

    if (read_select && _hasReadyRx()) {
        need_block = false;
    }

    if (need_block) {
        if (_down) {
            return false;
        } else {
            _pollfd.fd = _fd;
            _pollfd.events |= POLLIN;
            stats.num_rx_poll_req++;
            if (_hasReadyTx() && write_select) {
                _pollfd.events |= POLLOUT;
                stats.num_tx_poll_req++;
            }
        }
        if (_evt_handle != nullptr && blocking_deadline > AP_HAL::native_micros64()) {
            _evt_handle->wait(blocking_deadline - AP_HAL::native_micros64());
        }
    }

    // Writing the output masks
    if (!_down) {
        write_select = true;     // Always ready to write if not down
    } else {
        write_select = false;
    }
    if (_hasReadyRx()) {
        read_select = true;      // Readability depends only on RX buf, even if down
    } else {
        read_select = false;
    }

    // Return value is irrelevant as long as it's non-negative
    return true;
}

bool CANIface::set_event_handle(AP_HAL::EventHandle* handle) {
    _evt_handle = handle;
    evt_can_socket[_self_index]._ifaces[_self_index] = this;
    _evt_handle->set_source(&evt_can_socket[_self_index]);
    return true;
}


bool CANIface::CANSocketEventSource::wait(uint64_t duration, AP_HAL::EventHandle* evt_handle)
{
    if (evt_handle == nullptr) {
        return false;
    }
    pollfd pollfds[HAL_NUM_CAN_IFACES] {};
    uint8_t pollfd_iface_map[HAL_NUM_CAN_IFACES] {};
    unsigned long int num_pollfds = 0;
    
    // Poll FD set setup
    for (unsigned i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (_ifaces[i] == nullptr) {
            continue;
        }
        if (_ifaces[i]->_down) {
            continue;
        }
        pollfds[num_pollfds] = _ifaces[i]->_pollfd;
        pollfd_iface_map[num_pollfds] = i;
        num_pollfds++;
        _ifaces[i]->stats.num_poll_waits++;
    }

    if (num_pollfds == 0) {
        return true;
    }

    // Timeout conversion
    auto ts = timespec();
    ts.tv_sec = duration / 1000000LL;
    ts.tv_nsec = (duration % 1000000LL) * 1000;

    // Blocking here
    const int res = ppoll(pollfds, num_pollfds, &ts, nullptr);

    if (res < 0) {
        return false;
    }

    // Handling poll output
    for (unsigned i = 0; i < num_pollfds; i++) {
        if (_ifaces[pollfd_iface_map[i]] == nullptr) {
            continue;
        }
        _ifaces[pollfd_iface_map[i]]->_updateDownStatusFromPollResult(pollfds[i]);

        const bool poll_read  = pollfds[i].revents & POLLIN;
        const bool poll_write = pollfds[i].revents & POLLOUT;
        _ifaces[pollfd_iface_map[i]]->_poll(poll_read, poll_write);
    }
    return true;
}

void CANIface::get_stats(ExpandingString &str)
{
    str.printf("tx_requests:    %u\n"
               "tx_rejected:    %u\n"
               "tx_full:        %u\n"
               "tx_confirmed:   %u\n"
               "tx_success:     %u\n"
               "tx_timedout:    %u\n"
               "rx_received:    %u\n"
               "rx_errors:      %u\n"
               "num_downs:      %u\n"
               "num_rx_poll_req:  %u\n"
               "num_tx_poll_req:  %u\n"
               "num_poll_waits:   %u\n"
               "num_poll_tx_events: %u\n"
               "num_poll_rx_events: %u\n",
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_full,
               stats.tx_confirmed,
               stats.tx_success,
               stats.tx_timedout,
               stats.rx_received,
               stats.rx_errors,
               stats.num_downs,
               stats.num_rx_poll_req,
               stats.num_tx_poll_req,
               stats.num_poll_waits,
               stats.num_poll_tx_events,
               stats.num_poll_rx_events);
}

#endif
