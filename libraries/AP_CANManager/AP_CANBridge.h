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
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include "AP_HAL/utility/RingBuffer.h"
#include <AP_Param/AP_Param.h>

namespace CANBridge
{

class CANIface: public AP_HAL::CANIface
{
protected:
    AP_HAL::CANIface* _can_iface; // Can interface to be used for interaction by SLCAN interface
    int8_t _iface_num = -1;
    HAL_Semaphore port_sem;

public:
    CANIface(AP_HAL::CANIface* can_iface, uint8_t iface_num) :
    _can_iface(can_iface), _iface_num(iface_num)
    {}

    CANIface() {}

    bool init(const uint32_t bitrate, const OperatingMode mode) override
    {
        return false;
    }

    int8_t get_iface_num() const
    {
        return _iface_num;
    }

    // Overriden methods
    bool set_event_handle(AP_HAL::EventHandle* evt_handle) override;
    uint16_t getNumFilters() const override;
    uint32_t getErrorCount() const override;
    void get_stats(ExpandingString &) override;
    bool is_busoff() const override;
    bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs) override;
    void flush_tx() override;
    void clear_rx() override;
    bool is_initialized() const override;
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 AP_HAL::CANIface::CanIOFlags flags) override;

    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                    AP_HAL::CANIface::CanIOFlags& out_flags) override;
};

}

#endif
