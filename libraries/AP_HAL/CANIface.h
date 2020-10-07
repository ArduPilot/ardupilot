/*
 * Copyright (C) 2020 Siddharth B Purohit
 *
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
 */

#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
struct AP_HAL::CANFrame {
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

    static const uint8_t MaxDataLen = 8;

    uint32_t id;                ///< CAN ID with flags (above)
    uint8_t data[MaxDataLen];
    uint8_t dlc;                ///< Data Length Code

    CANFrame() :
        id(0),
        dlc(0)
    {
        memset(data,0, MaxDataLen);
    }

    CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len) :
        id(can_id),
        dlc((data_len > MaxDataLen) ? MaxDataLen : data_len)
    {
        if ((can_data == nullptr) || (data_len != dlc) || (dlc == 0)) {
            return;
        }
        memcpy(this->data, can_data, dlc);
    }

    bool operator!=(const CANFrame& rhs) const
    {
        return !operator==(rhs);
    }
    bool operator==(const CANFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    bool isExtended()                  const
    {
        return id & FlagEFF;
    }
    bool isRemoteTransmissionRequest() const
    {
        return id & FlagRTR;
    }
    bool isErrorFrame()                const
    {
        return id & FlagERR;
    }

    /**
     * CAN frame arbitration rules, particularly STD vs EXT:
     *     Marco Di Natale - "Understanding and using the Controller Area Network"
     *     http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     */
    bool priorityHigherThan(const CANFrame& rhs) const;
    bool priorityLowerThan(const CANFrame& rhs) const
    {
        return rhs.priorityHigherThan(*this);
    }
};

class AP_HAL::CANIface
{
public:

    enum OperatingMode {
        PassThroughMode,
        NormalMode,
        SilentMode,
        FilteredMode
    };

    OperatingMode get_operating_mode() { return mode_; }

    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;
    static const CanIOFlags AbortOnError = 2;

    // Single Rx Frame with related info
    struct CanRxItem {
        uint64_t timestamp_us = 0;
        CanIOFlags flags = 0;
        CANFrame frame;
    };

    // Single Tx Frame with related info
    struct CanTxItem {
        uint64_t deadline = 0;
        CANFrame frame;
        uint32_t index = 0;
        bool loopback:1;
        bool abort_on_error:1;
        bool aborted:1;
        bool pushed:1;
        bool setup:1;

        bool operator<(const CanTxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame)) {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame)) {
                return false;
            }
            return index > rhs.index;
        }
    };

    struct CanFilterConfig {
        uint32_t id = 0;
        uint32_t mask = 0;

        bool operator==(const CanFilterConfig& rhs) const
        {
            return rhs.id == id && rhs.mask == mask;
        }
    };

    // Initialise the interface with hardware configuration required to start comms.
    virtual bool init(const uint32_t bitrate, const OperatingMode mode) = 0;

    // Select method to notify user of Rx and Tx buffer state.
    // fill read select with true if a frame is available in Rx buffer
    // fill write select with true if space is available in Tx buffer
    // Also waits for Rx or Tx event depending on read_select and write_select values
    // passed to the method until timeout. Returns true if the Rx/Tx even occured
    // while waiting, false if timedout
    virtual bool select(bool &read_select, bool &write_select,
                        const CANFrame* const pending_tx, uint64_t timeout)
    {
        return false;
    }

    virtual bool set_event_handle(EventHandle* evt_handle)
    {
        return true;
    }

    // Put frame in queue to be sent, return negative if error occured, 0 if no space, and 1 if successful
    virtual int16_t send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags) = 0;

    // Non blocking receive frame that pops the frames received inside the buffer, return negative if error occured, 
    // 0 if no frame available, 1 if successful
    virtual int16_t receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags) = 0;

    //Configure filters so as to reject frames that are not going to be handled by us
    virtual bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs)
    {
        return 0;
    }

    
    //Number of available hardware filters.
    virtual uint16_t getNumFilters() const
    {
        return 0;
    }

    //Return Total Error Count generated so far
    virtual uint32_t getErrorCount() const
    {
        return 0;
    }

    //Get status info of the interface
    virtual uint32_t get_stats(char* data, uint32_t max_size)
    {
        return 0;
    }

    // return true if busoff was detected and not cleared
    virtual bool is_busoff() const
    {
        return false;
    }

    // Methods to be used only while testing CANBus
    // Not for normal operation or use case
    virtual void flush_tx() {}
    virtual void clear_rx() {}

    // return true if init was called and successful
    virtual bool is_initialized() const = 0;
protected:
    uint32_t bitrate_;
    OperatingMode mode_;
};
