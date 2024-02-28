/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Pavel Kirienko
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

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

#include "AP_HAL_ChibiOS.h"
# if defined(STM32H7XX) || defined(STM32G4)
#include "CANFDIface.h"
# else
#if HAL_NUM_CAN_IFACES
#include "bxcan.hpp"

#ifndef HAL_CAN_RX_QUEUE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class ChibiOS::CANIface : public AP_HAL::CANIface
{
    static constexpr unsigned long IDE = (0x40000000U); // Identifier Extension
    static constexpr unsigned long STID_MASK = (0x1FFC0000U); // Standard Identifier Mask
    static constexpr unsigned long EXID_MASK = (0x1FFFFFFFU); // Extended Identifier Mask
    static constexpr unsigned long RTR       = (0x20000000U); // Remote Transmission Request
    static constexpr unsigned long DLC_MASK  = (0x000F0000U); // Data Length Code

    struct CriticalSectionLocker {
        CriticalSectionLocker()
        {
            chSysLock();
        }
        ~CriticalSectionLocker()
        {
            chSysUnlock();
        }
    };

    struct Timings {
        uint16_t prescaler;
        uint8_t sjw;
        uint8_t bs1;
        uint8_t bs2;

        Timings()
            : prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    enum { NumTxMailboxes = 3 };
    enum { NumFilters = 14 };
    static const uint32_t TSR_ABRQx[NumTxMailboxes];

    ChibiOS::bxcan::CanType* can_;

    // state for ISR RX handler. We put this in the class to avoid
    // having to expand the stack size for all threads
    AP_HAL::CANFrame isr_rx_frame;
    CanRxItem isr_rx_item;

    CanRxItem rx_buffer[HAL_CAN_RX_QUEUE_SIZE];
    ByteBuffer rx_bytebuffer_;
    ObjectBuffer<CanRxItem> rx_queue_;
    CanTxItem pending_tx_[NumTxMailboxes];
    bool irq_init_:1;
    bool initialised_:1;
    bool had_activity_:1;
    AP_HAL::BinarySemaphore *sem_handle;

    const uint8_t self_index_;

    bool computeTimings(uint32_t target_bitrate, Timings& out_timings);

    void setupMessageRam(void);

    bool readRxFIFO(uint8_t fifo_index);

    void discardTimedOutTxMailboxes(uint64_t current_time);

    bool canAcceptNewTxFrame(const AP_HAL::CANFrame& frame) const;

    bool isRxBufferEmpty() const;

    bool recover_from_busoff();

    void pollErrorFlags();

    void checkAvailable(bool& read, bool& write,
                        const AP_HAL::CANFrame* pending_tx) const;

    bool waitMsrINakBitStateChange(bool target_state);

    void handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t timestamp_us);

    void initOnce(bool enable_irq);

#if !defined(HAL_BOOTLOADER_BUILD)
    /*
      additional statistics
     */
    struct bus_stats : public AP_HAL::CANIface::bus_stats_t {
        uint32_t num_events;
        uint32_t esr;
    } stats;
#endif

public:
    /******************************************
     *   Common CAN methods                   *
     * ****************************************/
    CANIface(uint8_t index);
    CANIface();
    static uint8_t next_interface;

    // Initialise CAN Peripheral
    bool init(const uint32_t bitrate, const OperatingMode mode) override;

    // Put frame into Tx FIFO returns negative on error, 0 on buffer full, 
    // 1 on successfully pushing a frame into FIFO
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 CanIOFlags flags) override;

    // Receive frame from Rx Buffer, returns negative on error, 0 on nothing available, 
    // 1 on successfully poping a frame
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us,
                    CanIOFlags& out_flags) override;

#if !defined(HAL_BOOTLOADER_BUILD)
    // Set Filters to ignore frames not to be handled by us
    bool configureFilters(const CanFilterConfig* filter_configs,
                          uint16_t num_configs) override;
#endif
    // In BxCAN the Busoff error is cleared automatically,
    // so always return false
    bool is_busoff() const override
    {
        return false;
    }

    void clear_rx() override;

    // Get number of Filter configurations
    uint16_t getNumFilters() const override
    {
        return NumFilters;
    }

    // Get total number of Errors discovered
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    uint32_t getErrorCount() const override;
#endif

    // returns true if init was successfully called
    bool is_initialized() const override
    {
        return initialised_;
    }

    /******************************************
     * Select Method                          *
     * ****************************************/
    // wait until selected event is available, false when timed out waiting else true
    bool select(bool &read, bool &write,
                const AP_HAL::CANFrame* const pending_tx,
                uint64_t blocking_deadline) override;
    
    // setup event handle for waiting on events
    bool set_event_handle(AP_HAL::BinarySemaphore *handle) override;

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    // fetch stats text and return the size of the same,
    // results available via @SYS/can0_stats.txt or @SYS/can1_stats.txt 
    void get_stats(ExpandingString &str) override;
#endif

#if !defined(HAL_BOOTLOADER_BUILD)
    /*
      return statistics structure
     */
    const bus_stats_t *get_statistics(void) const override {
        return &stats;
    }
#endif

    /************************************
     * Methods used inside interrupt    *
     ************************************/
    void handleTxInterrupt(uint64_t timestamp_us);
    void handleRxInterrupt(uint8_t fifo_index, uint64_t timestamp_us);
    
    // handle if any error occured, and do the needful such as,
    // droping the frame, and counting errors
    void pollErrorFlagsFromISR(void);

    // CAN Peripheral register structure
    static constexpr bxcan::CanType* const Can[HAL_NUM_CAN_IFACES] = { HAL_CAN_BASE_LIST };

protected:
    bool add_to_rx_queue(const CanRxItem &rx_item) override {
        return rx_queue_.push(rx_item);
    }

    int8_t get_iface_num(void) const override {
        return self_index_;
    }
};
#endif //HAL_NUM_CAN_IFACES
#endif //# if defined(STM32H7XX) || defined(STM32G4)
