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

#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_WITH_UAVCAN

#include "CAN.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

uavcan::MonotonicTime getMonotonic()
{
    return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
}

static can_frame makeSocketCanFrame(const uavcan::CanFrame& uavcan_frame)
{
    can_frame sockcan_frame { uavcan_frame.id& uavcan::CanFrame::MaskExtID, uavcan_frame.dlc, { } };
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

static uavcan::CanFrame makeUavcanFrame(const can_frame& sockcan_frame)
{
    uavcan::CanFrame uavcan_frame(sockcan_frame.can_id & CAN_EFF_MASK, sockcan_frame.data, sockcan_frame.can_dlc);
    if (sockcan_frame.can_id & CAN_EFF_FLAG) {
        uavcan_frame.id |= uavcan::CanFrame::FlagEFF;
    }
    if (sockcan_frame.can_id & CAN_ERR_FLAG) {
        uavcan_frame.id |= uavcan::CanFrame::FlagERR;
    }
    if (sockcan_frame.can_id & CAN_RTR_FLAG) {
        uavcan_frame.id |= uavcan::CanFrame::FlagRTR;
    }
    return uavcan_frame;
}

bool CAN::begin(uint32_t bitrate)
{
    if (_initialized) {
        return _initialized;
    }

    // TODO: Add possibility change bitrate
    _fd = openSocket(HAL_BOARD_CAN_IFACE_NAME);
    if (_fd > 0) {
        _bitrate = bitrate;
        _initialized = true;
    } else {
        _initialized = false;
    }
    return _initialized;
}

void CAN::reset()
{
    if (_initialized && _bitrate != 0) {
        close(_fd);
        begin(_bitrate);
    }
}

void CAN::end()
{
    _initialized = false;
    close(_fd);
}

bool CAN::is_initialized()
{
    return _initialized;
}

int32_t CAN::tx_pending()
{
    if (_initialized) {
        return _tx_queue.size();
    } else {
        return -1;
    }
}

int32_t CAN::available()
{
    if (_initialized) {
        return _rx_queue.size();
    } else {
        return -1;
    }
}

int CAN::openSocket(const std::string& iface_name)
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

int16_t CAN::send(const uavcan::CanFrame& frame, const uavcan::MonotonicTime tx_deadline,
                       const uavcan::CanIOFlags flags)
{
    _tx_queue.emplace(frame, tx_deadline, flags, _tx_frame_counter);
    _tx_frame_counter++;
    _pollRead();     // Read poll is necessary because it can release the pending TX flag
    _pollWrite();
    return 1;
}

int16_t CAN::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                          uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    if (_rx_queue.empty()) {
        _pollRead();            // This allows to use the socket not calling poll() explicitly.
        if (_rx_queue.empty()) {
            return 0;
        }
    }
    {
        const RxItem& rx = _rx_queue.front();
        out_frame        = rx.frame;
        out_ts_monotonic = rx.ts_mono;
        out_ts_utc       = rx.ts_utc;
        out_flags        = rx.flags;
    }
    _rx_queue.pop();
    return 1;
}

bool CAN::hasReadyTx() const
{
    return !_tx_queue.empty() && (_frames_in_socket_tx_queue < _max_frames_in_socket_tx_queue);
}

bool CAN::hasReadyRx() const
{
    return !_rx_queue.empty();
}

void CAN::poll(bool read, bool write)
{
    if (read) {
        _pollRead();  // Read poll must be executed first because it may decrement _frames_in_socket_tx_queue
    }
    if (write) {
        _pollWrite();
    }
}

int16_t CAN::configureFilters(const uavcan::CanFilterConfig* const filter_configs,
                              const std::uint16_t num_configs)
{
    if (filter_configs == nullptr) {
        return -1;
    }
    _hw_filters_container.clear();
    _hw_filters_container.resize(num_configs);

    for (unsigned i = 0; i < num_configs; i++) {
        const uavcan::CanFilterConfig& fc = filter_configs[i];
        _hw_filters_container[i].can_id   = fc.id   & uavcan::CanFrame::MaskExtID;
        _hw_filters_container[i].can_mask = fc.mask & uavcan::CanFrame::MaskExtID;
        if (fc.id & uavcan::CanFrame::FlagEFF) {
            _hw_filters_container[i].can_id |= CAN_EFF_FLAG;
        }
        if (fc.id & uavcan::CanFrame::FlagRTR) {
            _hw_filters_container[i].can_id |= CAN_RTR_FLAG;
        }
        if (fc.mask & uavcan::CanFrame::FlagEFF) {
            _hw_filters_container[i].can_mask |= CAN_EFF_FLAG;
        }
        if (fc.mask & uavcan::CanFrame::FlagRTR) {
            _hw_filters_container[i].can_mask |= CAN_RTR_FLAG;
        }
    }

    return 0;
}

/**
 * SocketCAN emulates the CAN filters in software, so the number of filters is virtually unlimited.
 * This method returns a constant value.
 */
static constexpr unsigned NumFilters = CAN_FILTER_NUMBER;
uint16_t CAN::getNumFilters() const { return NumFilters; }

uint64_t CAN::getErrorCount() const
{
    uint64_t ec = 0;
    for (auto& kv : _errors) { ec += kv.second; }
    return ec;
}

void CAN::_pollWrite()
{
    while (hasReadyTx()) {
        const TxItem tx = _tx_queue.top();

        if (tx.deadline >= getMonotonic()) {
            const int res = _write(tx.frame);
            if (res == 1) {                   // Transmitted successfully
                _incrementNumFramesInSocketTxQueue();
                if (tx.flags & uavcan::CanIOFlagLoopback) {
                    _pending_loopback_ids.insert(tx.frame.id);
                }
            } else if (res == 0) {            // Not transmitted, nor is it an error
                break;                        // Leaving the loop, the frame remains enqueued for the next retry
            } else {                          // Transmission error
                _registerError(SocketCanError::SocketWriteFailure);
            }
        } else {
            _registerError(SocketCanError::TxTimeout);
        }

        // Removing the frame from the queue even if transmission failed
        _tx_queue.pop();
    }
}

void CAN::_pollRead()
{
    uint8_t iterations_count = 0;
    while (iterations_count < CAN_MAX_POLL_ITERATIONS_COUNT)
    {
        iterations_count++;
        RxItem rx;
        rx.ts_mono = getMonotonic();  // Monotonic timestamp is not required to be precise (unlike UTC)
        bool loopback = false;
        const int res = _read(rx.frame, rx.ts_utc, loopback);
        if (res == 1) {
            bool accept = true;
            if (loopback) {           // We receive loopback for all CAN frames
                _confirmSentFrame();
                rx.flags |= uavcan::CanIOFlagLoopback;
                accept = _wasInPendingLoopbackSet(rx.frame);
            }
            if (accept) {
                _rx_queue.push(rx);
            }
        } else if (res == 0) {
            break;
        } else {
            _registerError(SocketCanError::SocketReadFailure);
            break;
        }
    }
}

int CAN::_write(const uavcan::CanFrame& frame) const
{
    errno = 0;

    const can_frame sockcan_frame = makeSocketCanFrame(frame);

    const int res = write(_fd, &sockcan_frame, sizeof(sockcan_frame));
    if (res <= 0) {
        if (errno == ENOBUFS || errno == EAGAIN) {  // Writing is not possible atm, not an error
            return 0;
        }
        return res;
    }
    if (res != sizeof(sockcan_frame)) {
        return -1;
    }
    return 1;
}


int CAN::_read(uavcan::CanFrame& frame, uavcan::UtcTime& ts_utc, bool& loopback) const
{
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

    frame = makeUavcanFrame(sockcan_frame);
    /*
     * Timestamp
     */
    const cmsghdr* const cmsg = CMSG_FIRSTHDR(&msg);
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
        auto tv = timeval();
        std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));  // Copy to avoid alignment problems
        ts_utc = uavcan::UtcTime::fromUSec(std::uint64_t(tv.tv_sec) * 1000000ULL + tv.tv_usec);
    } else {
        return -1;
    }
    return 1;
}

void CAN::_incrementNumFramesInSocketTxQueue()
{
    _frames_in_socket_tx_queue++;
}

void CAN::_confirmSentFrame()
{
    if (_frames_in_socket_tx_queue > 0) {
        _frames_in_socket_tx_queue--;
    }
}

bool CAN::_wasInPendingLoopbackSet(const uavcan::CanFrame& frame)
{
    if (_pending_loopback_ids.count(frame.id) > 0) {
        _pending_loopback_ids.erase(frame.id);
        return true;
    }
    return false;
}

bool CAN::_checkHWFilters(const can_frame& frame) const
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

void CANManager::IfaceWrapper::updateDownStatusFromPollResult(const pollfd& pfd)
{
    if (!_down&& (pfd.revents & POLLERR)) {
        int error = 0;
        socklen_t errlen = sizeof(error);
        getsockopt(pfd.fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<void*>(&error), &errlen);

        _down= error == ENETDOWN || error == ENODEV;

        hal.console->printf("Iface %d is dead; error %d", this->getFileDescriptor(), error);
    }
}

void CANManager::_timer_tick()
{
    if (!_initialized) return;

    if (p_uavcan != nullptr) {
        p_uavcan->do_cyclic();
    } else {
        hal.console->printf("p_uavcan is nullptr");
    }
}

bool CANManager::begin(uint32_t bitrate, uint8_t can_number)
{
    if (init(can_number) >= 0) {
        _initialized = true;
    }
    return _initialized;
}

bool CANManager::is_initialized()
{
    return _initialized;
}

void CANManager::initialized(bool val)
{
    _initialized = val;
}

AP_UAVCAN *CANManager::get_UAVCAN(void)
{
    return p_uavcan;
}

void CANManager::set_UAVCAN(AP_UAVCAN *uavcan)
{
    p_uavcan = uavcan;
}

CAN* CANManager::getIface(uint8_t iface_index)
{
    return (iface_index >= _ifaces.size()) ? nullptr : _ifaces[iface_index].get();
}

int CANManager::init(uint8_t can_number)
{
    int res = -1;
    char iface_name[16];
    sprintf(iface_name, "can%u", can_number);

    res = addIface(iface_name);
    if (res < 0) {
        hal.console->printf("CANManager: init %s failed\n", iface_name);
    }

    return res;
}

int16_t CANManager::select(uavcan::CanSelectMasks& inout_masks,
                    const uavcan::CanFrame* (&)[uavcan::MaxCanIfaces],
                    uavcan::MonotonicTime blocking_deadline)
{
    // Detecting whether we need to block at all
    bool need_block = (inout_masks.write == 0);    // Write queue is infinite

    for (unsigned i = 0; need_block && (i < _ifaces.size()); i++) {
        const bool need_read = inout_masks.read  & (1 << i);
        if (need_read && _ifaces[i]->hasReadyRx()) {
            need_block = false;
        }
    }

    if (need_block) {
        // Poll FD set setup
        pollfd pollfds[uavcan::MaxCanIfaces] = {};
        unsigned num_pollfds = 0;
        IfaceWrapper* pollfd_index_to_iface[uavcan::MaxCanIfaces] = {};

        for (unsigned i = 0; i < _ifaces.size(); i++) {
            if (_ifaces[i]->isDown()) {
                continue;
            }
            pollfds[num_pollfds].fd = _ifaces[i]->getFileDescriptor();
            pollfds[num_pollfds].events = POLLIN;
            if (_ifaces[i]->hasReadyTx() || (inout_masks.write & (1U << i))) {
                pollfds[num_pollfds].events |= POLLOUT;
            }
            pollfd_index_to_iface[num_pollfds] = _ifaces[i].get();
            num_pollfds++;
        }

        if (num_pollfds == 0) {
            return 0;
        }

        // Timeout conversion
        const std::int64_t timeout_usec = (blocking_deadline - getMonotonic()).toUSec();
        auto ts = timespec();
        if (timeout_usec > 0) {
            ts.tv_sec = timeout_usec / 1000000LL;
            ts.tv_nsec = (timeout_usec % 1000000LL) * 1000;
        }

        // Blocking here
        const int res = ppoll(pollfds, num_pollfds, &ts, nullptr);
        if (res < 0) {
            return res;
        }

        // Handling poll output
        for (unsigned i = 0; i < num_pollfds; i++) {
            pollfd_index_to_iface[i]->updateDownStatusFromPollResult(pollfds[i]);

            const bool poll_read  = pollfds[i].revents & POLLIN;
            const bool poll_write = pollfds[i].revents & POLLOUT;
            pollfd_index_to_iface[i]->poll(poll_read, poll_write);
        }
    }

    // Writing the output masks
    inout_masks = uavcan::CanSelectMasks();
    for (unsigned i = 0; i < _ifaces.size(); i++) {
        if (!_ifaces[i]->isDown()) {
            inout_masks.write |= std::uint8_t(1U << i);     // Always ready to write if not down
        }
        if (_ifaces[i]->hasReadyRx()) {
            inout_masks.read |= std::uint8_t(1U << i);      // Readability depends only on RX buf, even if down
        }
    }

    // Return value is irrelevant as long as it's non-negative
    return _ifaces.size();
}

int CANManager::addIface(const std::string& iface_name)
{
    if (_ifaces.size() >= uavcan::MaxCanIfaces) {
        return -1;
    }

    // Open the socket
    const int fd = CAN::openSocket(iface_name);
    if (fd < 0) {
        return fd;
    }

    // Construct the iface - upon successful construction the iface will take ownership of the fd.
    _ifaces.emplace_back(new IfaceWrapper(fd));

    hal.console->printf("New iface '%s' fd %d\n", iface_name.c_str(), fd);

    return _ifaces.size() - 1;
}

#endif
