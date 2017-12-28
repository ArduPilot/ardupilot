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

#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/CAN.h>

#include <linux/can.h>

#include <string>
#include <queue>
#include <memory>
#include <map>
#include <unordered_set>
#include <poll.h>

namespace Linux {

enum class SocketCanError
{
    SocketReadFailure,
    SocketWriteFailure,
    TxTimeout
};

class LinuxCAN: public AP_HAL::CAN {
public:
    LinuxCAN(int socket_fd=0)
      : _fd(socket_fd)
      , _frames_in_socket_tx_queue(0)
      , _max_frames_in_socket_tx_queue(2)
    { }
    ~LinuxCAN() { }

    bool begin(uint32_t bitrate) override;

    void end() override;

    void reset() override;

    bool is_initialized() override;

    int32_t tx_pending() override;

    int32_t available() override;

    static int openSocket(const std::string& iface_name);

    int getFileDescriptor() const { return _fd; }

    int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                     uavcan::CanIOFlags flags) override;

    int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                        uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override;

    bool hasReadyTx() const;

    bool hasReadyRx() const;

    void poll(bool read, bool write);

    int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs) override;

    uint16_t getNumFilters() const override;

    uint64_t getErrorCount() const override;


private:
    struct TxItem
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime deadline;
        uavcan::CanIOFlags flags = 0;
        std::uint64_t order = 0;

        TxItem(const uavcan::CanFrame& arg_frame, uavcan::MonotonicTime arg_deadline,
               uavcan::CanIOFlags arg_flags, std::uint64_t arg_order)
            : frame(arg_frame)
            , deadline(arg_deadline)
            , flags(arg_flags)
            , order(arg_order)
        { }

        bool operator<(const TxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame)) {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame)) {
                return false;
            }
            return order > rhs.order;
        }
    };

    struct RxItem
    {
        uavcan::CanFrame frame;
        uavcan::MonotonicTime ts_mono;
        uavcan::UtcTime ts_utc;
        uavcan::CanIOFlags flags;

        RxItem()
            : flags(0)
        { }
    };

    void _pollWrite();

    void _pollRead();

    int _write(const uavcan::CanFrame& frame) const;

    int _read(uavcan::CanFrame& frame, uavcan::UtcTime& ts_utc, bool& loopback) const;

    void _incrementNumFramesInSocketTxQueue();

    void _confirmSentFrame();

    bool _wasInPendingLoopbackSet(const uavcan::CanFrame& frame);

    bool _checkHWFilters(const can_frame& frame) const;

    void _registerError(SocketCanError e) { _errors[e]++; }

    uint32_t _bitrate;

    bool _initialized;

    int _fd;

    const unsigned _max_frames_in_socket_tx_queue;
    unsigned _frames_in_socket_tx_queue;
    uint64_t _tx_frame_counter;

    std::map<SocketCanError, uint64_t> _errors;
    std::priority_queue<TxItem> _tx_queue;
    std::queue<RxItem> _rx_queue;
    std::unordered_multiset<uint32_t> _pending_loopback_ids;
    std::vector<can_filter> _hw_filters_container;
};

class LinuxCANDriver: public AP_HAL::CANManager {
public:
    static LinuxCANDriver *from(AP_HAL::CANManager *can)
    {
        return static_cast<LinuxCANDriver*>(can);
    }

    LinuxCANDriver() : _initialized(false) { _ifaces.reserve(uavcan::MaxCanIfaces); }
    ~LinuxCANDriver() { }

    void register_thread();

    void _timer_tick();

    bool begin(uint32_t bitrate, uint8_t can_number) override;

    bool is_initialized() override;

    void initialized(bool val) override;

    AP_UAVCAN *p_uavcan;

    AP_UAVCAN *get_UAVCAN(void) override;

    void set_UAVCAN(AP_UAVCAN *uavcan) override;

    virtual LinuxCAN* getIface(uint8_t iface_index) override;

    virtual uint8_t getNumIfaces() const override { return _ifaces.size(); }

    int init(uint8_t can_number);

    virtual int16_t select(uavcan::CanSelectMasks& inout_masks,
                   const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], uavcan::MonotonicTime blocking_deadline) override;

    int addIface(const std::string& iface_name);

private:
    class IfaceWrapper : public LinuxCAN
    {
        bool down_ = false;

    public:
        IfaceWrapper(int fd) : LinuxCAN(fd) { }

        void updateDownStatusFromPollResult(const pollfd& pfd);

        bool isDown() const { return down_; }
    };

    bool _initialized;

    std::vector<std::unique_ptr<IfaceWrapper>> _ifaces;
};

}
