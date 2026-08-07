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
#include "AP_HAL_Boards.h"

class ExpandingString;

/**
 * Raw CAN frame, as passed to/from the CAN driver.
 */
struct AP_HAL::CANFrame {
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

#if HAL_CANFD_SUPPORTED
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 64;
#else
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 8;
#endif
    uint32_t id;                ///< CAN ID with flags (above)
    union {
        uint8_t data[MaxDataLen];
        uint32_t data_32[MaxDataLen/4];
    };
    uint8_t dlc;                ///< Data Length Code
    bool canfd;

    CANFrame() :
        id(0),
        dlc(0),
        canfd(false)
    {
        memset(data,0, MaxDataLen);
    }

    CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame = false);

    bool operator!=(const CANFrame& rhs) const
    {
        return !operator==(rhs);
    }
    bool operator==(const CANFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    // signed version of id, for use by scriping where uint32_t is expensive
    int32_t id_signed(void) const {
        return isExtended()? int32_t(id & MaskExtID) : int32_t(id & MaskStdID);
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
    void setCanFD(bool canfd_frame)
    {
        canfd = canfd_frame;
    }

    bool isCanFDFrame() const
    {
        return canfd;
    }

    static uint8_t dlcToDataLength(uint8_t dlc);

    static uint8_t dataLengthToDlc(uint8_t data_length);
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

    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;
    static const CanIOFlags AbortOnError = 2;
    static const CanIOFlags IsForwardedFrame = 4;

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
        bool canfd_frame:1;

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

    virtual bool init(const uint32_t bitrate, const uint32_t fdbitrate) {
        return init(bitrate);
    }

    // Initialise the interface with hardware configuration required to start comms.
    virtual bool init(const uint32_t bitrate) = 0;

    // Select method to notify user of Rx and Tx buffer state.
    // fill read select with true if a frame is available in Rx buffer
    // fill write select with true if space is available in Tx buffer
    // Also waits for Rx or Tx event depending on read_select and write_select values
    // passed to the method until timeout. Returns true if the Rx/Tx even occurred
    // while waiting, false if timedout
    virtual bool select(bool &read_select, bool &write_select,
                        const CANFrame* const pending_tx, uint64_t timeout)
    {
        return false;
    }

    virtual bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle)
    {
        return true;
    }

    // Put frame in queue to be sent, return negative if error occurred, 0 if no space, and 1 if successful
    // must be called on child class
    virtual int16_t send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags);

    // Non blocking receive frame that pops the frames received inside the buffer, return negative if error occurred, 
    // 0 if no frame available, 1 if successful
    // must be called on child class
    virtual int16_t receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags);

    //Return Total Error Count generated so far
    virtual uint32_t getErrorCount() const
    {
        return 0;
    }

    typedef struct {
        uint32_t tx_requests;
        uint32_t tx_rejected;
        uint32_t tx_overflow;
        uint32_t tx_success;
        uint32_t tx_timedout;
        uint32_t tx_abort;
        uint32_t rx_received;
        uint32_t rx_overflow;
        uint32_t rx_errors;
        uint32_t num_busoff_err;
        uint64_t last_transmit_us;
    } bus_stats_t;

#if !defined(HAL_BOOTLOADER_BUILD)
    //Get status info of the interface
    virtual void get_stats(ExpandingString &str) {}

    /*
      return bus statistics for logging
      return nullptr if no statistics available
     */
    virtual const bus_stats_t *get_statistics(void) const { return nullptr; };
#endif

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

    FUNCTOR_TYPEDEF(FrameCb, void, uint8_t, const AP_HAL::CANFrame &, CanIOFlags);

    // register a frame callback function
    virtual bool register_frame_callback(FrameCb cb, uint8_t &cb_id);
    virtual void unregister_frame_callback(uint8_t cb_id);

protected:
    virtual int8_t get_iface_num() const = 0;
    virtual bool add_to_rx_queue(const CanRxItem &rx_item) = 0;

    struct {
#ifndef HAL_BOOTLOADER_BUILD
        HAL_Semaphore sem;
#endif
        // allow up to 3 callbacks per interface
        FrameCb cb[3];
    } callbacks;

    uint32_t bitrate_;
};
