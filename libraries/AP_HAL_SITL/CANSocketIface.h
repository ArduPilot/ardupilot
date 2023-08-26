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

#include "AP_HAL_SITL.h"

#if HAL_NUM_CAN_IFACES

#include <AP_HAL/CANIface.h>

#include <string>
#include <queue>
#include <memory>
#include <map>
#include <unordered_set>
#include <poll.h>
#include "CAN_Transport.h"

namespace HALSITL {

class CANIface: public AP_HAL::CANIface {
public:
    CANIface(int index)
      : _self_index(index)
      , _frames_in_socket_tx_queue(0)
    {
        _num_interfaces++;
    }
    CANIface() : CANIface(_num_interfaces) {}
    static uint8_t _num_interfaces;

    ~CANIface() { }

    // Initialise CAN Peripheral
    bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) override;
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    // number of enabled interfaces
    static uint8_t num_interfaces(void) {
        return _num_interfaces;
    }

    // Put frame into Tx FIFO returns negative on error, 0 on buffer full, 
    // 1 on successfully pushing a frame into FIFO
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // Receive frame from Rx Buffer, returns negative on error, 0 on nothing available, 
    // 1 on successfully poping a frame
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

    // Always return false, there's no busoff condition in virtual CAN
    bool is_busoff() const override
    {
        return false;
    }

    void flush_tx() override;

    void clear_rx() override;

    // Get total number of Errors discovered
    uint32_t getErrorCount() const override;

    // returns true if init was successfully called
    bool is_initialized() const override;

    /******************************************
     * Select Method                          *
     * ****************************************/
    // wait until selected event is available, false when timed out waiting else true
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    
    // setup event handle for waiting on events
    bool set_event_handle(AP_HAL::EventHandle* handle) override;

    // fetch stats text and return the size of the same,
    // results available via @SYS/can0_stats.txt or @SYS/can1_stats.txt 
    void get_stats(ExpandingString &str) override;

    /*
      return statistics structure
     */
    const bus_stats_t *get_statistics(void) const override {
        return &stats;
    }
    
    class CANSocketEventSource : public AP_HAL::EventSource {
        friend class CANIface;
        CANIface *_ifaces[HAL_NUM_CAN_IFACES];
        
    public:
        // we just poll fd, no signaling is done
        void signal(uint32_t evt_mask) override { return; }
        bool wait(uint16_t duration_us, AP_HAL::EventHandle* evt_handle) override;
    };

private:
    void _pollWrite();

    bool _pollRead();

    void _confirmSentFrame();

    bool _hasReadyTx();

    bool _hasReadyRx();

    void _poll(bool read, bool write);

    int _openSocket(const std::string& iface_name);

    void _updateDownStatusFromPollResult(const pollfd& pfd);

    CAN_Transport *transport;

    const uint8_t _self_index;

    unsigned _frames_in_socket_tx_queue;
    uint32_t _tx_frame_counter;
    AP_HAL::EventHandle *_evt_handle;
    static CANSocketEventSource evt_can_socket[HAL_NUM_CAN_IFACES];

    pollfd _pollfd;
    std::priority_queue<CanTxItem> _tx_queue;
    std::queue<CanRxItem> _rx_queue;

    /*
      bus statistics
     */
    AP_HAL::CANIface::bus_stats_t stats;

    HAL_Semaphore sem;

    bool add_to_rx_queue(const CanRxItem &rx_item) override {
        _rx_queue.push(rx_item);
        return true;
    }

    int8_t get_iface_num(void) const override {
        return _self_index;
    }
};

}

#endif //#if HAL_NUM_CAN_IFACES
