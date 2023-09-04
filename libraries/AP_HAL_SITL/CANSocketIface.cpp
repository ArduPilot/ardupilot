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

#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include "Scheduler.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>
#include "CAN_Multicast.h"
#include "CAN_SocketCAN.h"

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANSITLIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

CANIface::CANSocketEventSource CANIface::evt_can_socket[HAL_NUM_CAN_IFACES];

uint8_t CANIface::_num_interfaces;

bool CANIface::is_initialized() const
{
    return transport != nullptr;
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
        _pollRead(); // Read poll must be executed first because it may decrement _frames_in_socket_tx_queue
    }
    if (write) {
        _pollWrite();
    }
}

uint32_t CANIface::getErrorCount() const
{
    return 0;
}

void CANIface::_pollWrite()
{
    if (transport == nullptr) {
        return;
    }
    while (_hasReadyTx()) {
        WITH_SEMAPHORE(sem);
        const CanTxItem tx = _tx_queue.top();
        const uint64_t curr_time = AP_HAL::micros64();
        if (tx.deadline >= curr_time) {
            // hal.console->printf("%x TDEAD: %lu CURRT: %lu DEL: %lu\n",tx.frame.id,  tx.deadline, curr_time, tx.deadline-curr_time);
            bool ok = transport->send(tx.frame);
            if (ok) {
                stats.tx_success++;
                stats.last_transmit_us = curr_time;
            } else {
                break;
            }
        } else {
            stats.tx_timedout++;
        }

        // Removing the frame from the queue
        (void)_tx_queue.pop();
    }
}

bool CANIface::_pollRead()
{
    if (transport == nullptr) {
        return false;
    }
    CanRxItem rx {};
    bool ok = transport->receive(rx.frame);
    if (!ok) {
        return false;
    }
    rx.timestamp_us = AP_HAL::micros64();
    WITH_SEMAPHORE(sem);
    add_to_rx_queue(rx);
    stats.rx_received++;
    return true;
}

// Might block forever, only to be used for testing
void CANIface::flush_tx()
{
    WITH_SEMAPHORE(sem);
    do {
        _poll(true, true);
    } while(!_tx_queue.empty());
}

void CANIface::clear_rx()
{
    WITH_SEMAPHORE(sem);
    // Clean Rx Queue
    std::queue<CanRxItem> empty;
    std::swap( _rx_queue, empty );
}

void CANIface::_confirmSentFrame()
{
    if (_frames_in_socket_tx_queue > 0) {
        _frames_in_socket_tx_queue--;
    }
}

bool CANIface::init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode)
{
    return init(bitrate, mode);
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    const auto *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return false;
    }
    if (_self_index >= HAL_NUM_CAN_IFACES) {
        return false;
    }
    const SITL::SIM::CANTransport can_type = _sitl->can_transport[_self_index];
    switch (can_type) {
    case SITL::SIM::CANTransport::MulticastUDP:
        transport = new CAN_Multicast();
        break;
    case SITL::SIM::CANTransport::SocketCAN:
#if HAL_CAN_WITH_SOCKETCAN
        transport = new CAN_SocketCAN();
#endif
        break;
    }
    if (transport == nullptr) {
        return false;
    }
    if (!transport->init(_self_index)) {
        delete transport;
        transport = nullptr;
        return false;
    }
    return true;
}

bool CANIface::select(bool &read_select, bool &write_select,
                      const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline)
{
    if (transport == nullptr) {
        return false;
    }
    // Detecting whether we need to block at all
    bool need_block = !write_select;    // Write queue is infinite

    // call poll here to flush some tx
    _poll(true, true);

    if (read_select && _hasReadyRx()) {
        need_block = false;
    }

    if (need_block) {
        _pollfd.fd = transport->get_read_fd();
        _pollfd.events |= POLLIN;
    }
    if (_evt_handle != nullptr && blocking_deadline > AP_HAL::micros64()) {
        _evt_handle->wait(blocking_deadline - AP_HAL::micros64());
    }

    // Writing the output masks
    write_select = true;
    read_select = _hasReadyRx();

    return true;
}

bool CANIface::set_event_handle(AP_HAL::EventHandle* handle)
{
    _evt_handle = handle;
    evt_can_socket[_self_index]._ifaces[_self_index] = this;
    _evt_handle->set_source(&evt_can_socket[_self_index]);
    return true;
}


bool CANIface::CANSocketEventSource::wait(uint16_t duration_us, AP_HAL::EventHandle* evt_handle)
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
        pollfds[num_pollfds] = _ifaces[i]->_pollfd;
        pollfd_iface_map[num_pollfds] = i;
        num_pollfds++;
    }

    if (num_pollfds == 0) {
        return true;
    }

    const uint32_t start_us = AP_HAL::micros();
    do {
        uint16_t wait_us = MIN(100, duration_us);

        // check FD for input
        const int res = poll(pollfds, num_pollfds, wait_us/1000U);

        if (res < 0) {
            return false;
        }
        if (res > 0) {
            break;
        }
        
        // ensure simulator runs
        hal.scheduler->delay_microseconds(wait_us);
    } while (AP_HAL::micros() - start_us < duration_us);


    // Handling poll output
    for (unsigned i = 0; i < num_pollfds; i++) {
        if (_ifaces[pollfd_iface_map[i]] == nullptr) {
            continue;
        }

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
               "tx_success:     %u\n"
               "tx_timedout:    %u\n"
               "rx_received:    %u\n"
               "rx_errors:      %u\n",
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_success,
               stats.tx_timedout,
               stats.rx_received,
               stats.rx_errors);
}

#endif
