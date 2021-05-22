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

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace SLCAN
{

class CANIface: public AP_HAL::CANIface
{

    int16_t reportFrame(const AP_HAL::CANFrame& frame, uint64_t timestamp_usec);

    const char* processCommand(char* cmd);

    // pushes received frame into queue, false if failed
    bool push_Frame(AP_HAL::CANFrame &frame);

    // Methods to handle different types of frames,
    // return true if successfully received frame type
    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd);

    // Parsing bytes received on the serial port
    inline void addByte(const uint8_t byte);

    // track changes to slcan serial port
    void update_slcan_port();

    bool initialized_;

    char buf_[SLCAN_BUFFER_SIZE + 1]; // buffer to record raw frame nibbles before parsing
    uint32_t _pending_frame_size = 0; // holds the size of frame to be tx
    int16_t pos_ = 0; // position in the buffer recording nibble frames before parsing
    AP_HAL::UARTDriver* _port; // UART interface port reference to be used for SLCAN iface

    ObjectBuffer<AP_HAL::CANIface::CanRxItem> rx_queue_; // Parsed Rx Frame queue

    const uint32_t _serial_lock_key = 0x53494442; // Key used to lock UART port for use by slcan

    AP_Int8 _slcan_can_port;
    AP_Int8 _slcan_ser_port;
    AP_Int8 _slcan_timeout;
    AP_Int8 _slcan_start_delay;

    bool _slcan_start_req;
    uint32_t _slcan_start_req_time;
    int8_t _prev_ser_port;
    int8_t _iface_num = -1;
    uint32_t _last_had_activity;
    uint8_t num_tries;
    AP_HAL::CANIface* _can_iface; // Can interface to be used for interaction by SLCAN interface
    HAL_Semaphore port_sem;
    bool _set_by_sermgr;
public:
    CANIface():
        rx_queue_(HAL_CAN_RX_QUEUE_SIZE)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    bool init(const uint32_t bitrate, const OperatingMode mode) override
    {
        return false;
    }

    // Initialisation of SLCAN Passthrough method of operation
    bool init_passthrough(uint8_t i);

    void reset_params();
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
