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

#include "AP_HAL_ChibiOS.h"

#if HAL_NUM_CAN_IFACES
#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
# include <hal.h>
#include <AP_CANManager/AP_CANManager.h>
# if defined(STM32H7XX)
#include "CANFDIface.h"

#define FDCAN1_IT0_IRQHandler      STM32_FDCAN1_IT0_HANDLER
#define FDCAN1_IT1_IRQHandler      STM32_FDCAN1_IT1_HANDLER
#define FDCAN2_IT0_IRQHandler      STM32_FDCAN2_IT0_HANDLER
#define FDCAN2_IT1_IRQHandler      STM32_FDCAN2_IT1_HANDLER

#define FDCAN_FRAME_BUFFER_SIZE 4         // Buffer size for 8 bytes data field

//Message RAM Allocations in Word lengths
#define MAX_FILTER_LIST_SIZE 80U            //80 element Standard Filter List elements or 40 element Extended Filter List
#define FDCAN_NUM_RXFIFO0_SIZE 104U         //26 Frames
#define FDCAN_TX_FIFO_BUFFER_SIZE 128U      //32 Frames

#define MESSAGE_RAM_END_ADDR 0x4000B5FC

extern AP_HAL::HAL& hal;

static_assert(STM32_FDCANCLK <= 80U*1000U*1000U, "FDCAN clock must be max 80MHz");

using namespace ChibiOS;

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANFDIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

constexpr CANIface::CanType* const CANIface::Can[];
static ChibiOS::CANIface* can_ifaces[HAL_NUM_CAN_IFACES] = {nullptr};
#define REG_SET_TIMEOUT 250 // if it takes longer than 250ms for setting a register we have failed
static inline bool driver_initialised(uint8_t iface_index)
{
    if (iface_index >= HAL_NUM_CAN_IFACES) {
        return false;
    }
    if (can_ifaces[iface_index] == nullptr) {
        return false;
    }
    return true;
}

static inline void handleCANInterrupt(uint8_t iface_index, uint8_t line_index)
{
    if (!driver_initialised(iface_index)) {
        //Just reset all the interrupts and return
        CANIface::Can[iface_index]->IR = FDCAN_IR_RF0N;
        CANIface::Can[iface_index]->IR = FDCAN_IR_RF1N;
        CANIface::Can[iface_index]->IR = FDCAN_IR_TEFN;
        return;
    }
    if (line_index == 0) {
        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_RF0N) ||
            (CANIface::Can[iface_index]->IR & FDCAN_IR_RF0F)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
            can_ifaces[iface_index]->handleRxInterrupt(0);
        }
        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_RF1N) ||
            (CANIface::Can[iface_index]->IR & FDCAN_IR_RF1F)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
            can_ifaces[iface_index]->handleRxInterrupt(1);
        }
    } else {
        if (CANIface::Can[iface_index]->IR & FDCAN_IR_TC) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_TC;
            uint64_t timestamp_us = AP_HAL::micros64();
            if (timestamp_us > 0) {
                timestamp_us--;
            }
            can_ifaces[iface_index]->handleTxCompleteInterrupt(timestamp_us);
        }

        if ((CANIface::Can[iface_index]->IR & FDCAN_IR_BO)) {
            CANIface::Can[iface_index]->IR = FDCAN_IR_BO;
            can_ifaces[iface_index]->handleBusOffInterrupt();
        }
    }
    can_ifaces[iface_index]->pollErrorFlagsFromISR();
}

uint32_t CANIface::FDCANMessageRAMOffset_ = 0;

CANIface::CANIface(uint8_t index) :
    self_index_(index),
    rx_bytebuffer_((uint8_t*)rx_buffer, sizeof(rx_buffer)),
    rx_queue_(&rx_bytebuffer_)
{
    if (index >= HAL_NUM_CAN_IFACES) {
         AP_HAL::panic("Bad CANIface index.");
    } else {
        can_ = Can[index];
    }
}

void CANIface::handleBusOffInterrupt()
{
    _detected_bus_off = true;
}

bool CANIface::computeTimings(const uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return false;
    }

    /*
     * Hardware configuration
     */
    const uint32_t pclk = STM32_FDCANCLK;


    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    struct BsPair {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(uint8_t(bs1_bs2_sum - bs1)),
            sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {}

        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
        return false;
    }

    Debug("Timings: quanta/bit: %d, sample point location: %.1f%%\n",
          int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                        // Which means one
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);
    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    stats.tx_requests++;
    if (frame.isErrorFrame() || frame.dlc > 8 || !initialised_) {
        stats.tx_rejected++;
        return -1;
    }

    CriticalSectionLocker lock;

    /*
     * Seeking for an empty slot
     */
    uint8_t index;

    if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        stats.tx_rejected++;
        return 0;    //we don't have free space
    }
    index = ((can_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);

    // Copy Frame to RAM
    // Calculate Tx element address
    uint32_t* buffer = (uint32_t *)(MessageRam_.TxFIFOQSA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));

    //Setup Frame ID
    if (frame.isExtended()) {
        buffer[0] = (IDE | frame.id);
    } else {
        buffer[0] = (frame.id << 18);
    }
    if (frame.isRemoteTransmissionRequest()) {
        buffer[0] |= RTR;
    }
    //Write Data Length Code, and Message Marker
    buffer[1] =  frame.dlc << 16 | index << 24;

    // Write Frame to the message RAM
    buffer[2] = (uint32_t(frame.data[3]) << 24) |
                (uint32_t(frame.data[2]) << 16) |
                (uint32_t(frame.data[1]) << 8)  |
                (uint32_t(frame.data[0]) << 0);
    buffer[3] = (uint32_t(frame.data[7]) << 24) |
                (uint32_t(frame.data[6]) << 16) |
                (uint32_t(frame.data[5]) << 8)  |
                (uint32_t(frame.data[4]) << 0);
    //Set Add Request
    can_->TXBAR = (1 << index);

    //Registering the pending transmission so we can track its deadline and loopback it as needed
    pending_tx_[index].deadline       = tx_deadline;
    pending_tx_[index].frame          = frame;
    pending_tx_[index].loopback       = (flags & AP_HAL::CANIface::Loopback) != 0;
    pending_tx_[index].abort_on_error = (flags & AP_HAL::CANIface::AbortOnError) != 0;
    pending_tx_[index].index          = index;
    // setup frame initial state
    pending_tx_[index].aborted        = false;
    pending_tx_[index].setup          = true;
    pending_tx_[index].pushed         = false;
    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    CriticalSectionLocker lock;
    CanRxItem rx_item;
    if (!rx_queue_.pop(rx_item) || !initialised_) {
        return 0;
    }
    out_frame    = rx_item.frame;
    out_timestamp_us = rx_item.timestamp_us;
    out_flags    = rx_item.flags;
    return 1;
}

bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
    uint32_t num_extid = 0, num_stdid = 0;
    uint32_t total_available_list_size = MAX_FILTER_LIST_SIZE;
    uint32_t* filter_ptr;
    if (initialised_ || mode_ != FilteredMode) {
        // we are already initialised can't do anything here
        return false;
    }
    //count number of frames of each type
    for (uint8_t i = 0; i < num_configs; i++) {
        const CanFilterConfig* const cfg = filter_configs + i;
        if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
            num_extid++;
        } else {
            num_stdid++;
        }
    }
    CriticalSectionLocker lock;
    //Allocate Message RAM for Standard ID Filter List
    if (num_stdid == 0) { //No Frame with Standard ID is to be accepted
        can_->GFC |= 0x2; //Reject All Standard ID Frames
    } else if ((num_stdid < total_available_list_size) && (num_stdid <= 128)) {
        can_->SIDFC = (FDCANMessageRAMOffset_ << 2) | (num_stdid << 16);
        MessageRam_.StandardFilterSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_stdid;
        total_available_list_size -= num_stdid;
        can_->GFC |= (0x3U << 4); //Reject non matching Standard frames
    } else {    //The List is too big, return fail
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }

    if (num_stdid) {
        num_stdid = 0;  //reset list count
        filter_ptr = (uint32_t*)MessageRam_.StandardFilterSA;
        //Run through the filter list and setup standard id filter list
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const CanFilterConfig* const cfg = filter_configs + i;
            if (!((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF))) {
                id   = (cfg->id   & AP_HAL::CANFrame::MaskStdID);  // Regular std frames, nothing fancy.
                mask = (cfg->mask & 0x7F);
                filter_ptr[num_stdid] = 0x2U << 30  | //Classic CAN Filter
                                        0x1U << 27 |  //Store in Rx FIFO0 if filter matches
                                        id << 16 |
                                        mask;
                num_stdid++;
            }
        }
    }

    //Allocate Message RAM for Extended ID Filter List
    if (num_extid == 0) { //No Frame with Extended ID is to be accepted
        can_->GFC |= 0x1; //Reject All Extended ID Frames
    } else if ((num_extid < (total_available_list_size/2)) && (num_extid <= 64)) {
        can_->XIDFC = (FDCANMessageRAMOffset_ << 2) | (num_extid << 16);
        MessageRam_.ExtendedFilterSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_extid*2;
        can_->GFC |= (0x3U << 2); // Reject non matching Extended frames
    } else {    //The List is too big, return fail
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }

    if (num_extid) {
        num_extid = 0;
        filter_ptr = (uint32_t*)MessageRam_.ExtendedFilterSA;
        //Run through the filter list and setup extended id filter list
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const CanFilterConfig* const cfg = filter_configs + i;
            if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
                id   = (cfg->id   & AP_HAL::CANFrame::MaskExtID);
                mask = (cfg->mask & AP_HAL::CANFrame::MaskExtID);
                filter_ptr[num_extid*2]       = 0x1U << 29 | id; //Store in Rx FIFO0 if filter matches
                filter_ptr[num_extid*2 + 1]   = 0x2U << 30 | mask; // Classic CAN Filter
                num_extid++;
            }
        }
    }

    MessageRam_.EndAddress = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    if (MessageRam_.EndAddress > MESSAGE_RAM_END_ADDR) {
        //We are overflowing the limit of Allocated Message RAM
        AP_HAL::panic("CANFDIface: Message RAM Overflow!");
    }

    // Finally get out of Config Mode
    can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
    uint32_t while_start_ms = AP_HAL::millis();
    while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
        if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
            return false;
        }
    }
    initialised_ = true;
    return true;
}

uint16_t CANIface::getNumFilters() const
{
    return MAX_FILTER_LIST_SIZE;
}

bool CANIface::clock_init_ = false;
bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    Debug("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    if (self_index_ > HAL_NUM_CAN_IFACES) {
        Debug("CAN drv init failed");
        return false;
    }
    if (can_ifaces[self_index_] == nullptr) {
        can_ifaces[self_index_] = this;
#if !defined(HAL_BOOTLOADER_BUILD)
        hal.can[self_index_] = this;
#endif
    }


    bitrate_ = bitrate;
    mode_ = mode;
    //Only do it once
    //Doing it second time will reset the previously initialised bus
    if (!clock_init_) {
        CriticalSectionLocker lock;
        RCC->APB1HENR  |= RCC_APB1HENR_FDCANEN;
        RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
        RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
        clock_init_ = true;
    }

    /*
     * IRQ
     */
    if (!irq_init_) {
        CriticalSectionLocker lock;
        if (self_index_ == 0) {
            nvicEnableVector(FDCAN1_IT0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(FDCAN1_IT1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
        }
# if HAL_NUM_CAN_IFACES > 1
        else if (self_index_ == 1) {
            nvicEnableVector(FDCAN2_IT0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(FDCAN2_IT1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
        }
# endif
        irq_init_ = true;
    }


    // Setup FDCAN for configuration mode and disable all interrupts
    {
        CriticalSectionLocker lock;

        can_->CCCR &= ~FDCAN_CCCR_CSR; // Exit sleep mode
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        } //Wait for wake up ack
        can_->CCCR |= FDCAN_CCCR_INIT; // Request init
        while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 0) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        can_->CCCR |= FDCAN_CCCR_CCE; //Enable Config change
        can_->IE = 0;                  // Disable interrupts while initialization is in progress
    }

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();
    for (uint32_t i=0; i < NumTxMailboxes; i++) {
        pending_tx_[i] = CanTxItem();
    }
    peak_tx_mailbox_index_ = 0;
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    Timings timings;

    if (!computeTimings(bitrate, timings)) {
        can_->CCCR &= ~FDCAN_CCCR_INIT;
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }
        return false;
    }
    Debug("Timings: presc=%u sjw=%u bs1=%u bs2=%u\n",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    //setup timing register
    //TODO: Do timing calculations for FDCAN
    can_->NBTP = ((timings.sjw << FDCAN_NBTP_NSJW_Pos)   |
                  (timings.bs1 << FDCAN_NBTP_NTSEG1_Pos) |
                  (timings.bs2 << FDCAN_NBTP_TSEG2_Pos)  |
                  (timings.prescaler << FDCAN_NBTP_NBRP_Pos));

    //RX Config
    can_->RXESC = 0; //Set for 8Byte Frames

    //Setup Message RAM
    setupMessageRam();
    // Reset Bus Off
    _detected_bus_off = false;
    //Clear all Interrupts
    can_->IR = 0x3FFFFFFF;
    //Enable Interrupts
    can_->IE =  FDCAN_IE_TCE |  // Transmit Complete interrupt enable
                FDCAN_IE_BOE |  // Bus off Error Interrupt enable
                FDCAN_IE_RF0NE |  // RX FIFO 0 new message
                FDCAN_IE_RF0FE |  // Rx FIFO 0 FIFO Full
                FDCAN_IE_RF1NE |  // RX FIFO 1 new message
                FDCAN_IE_RF1FE;   // Rx FIFO 1 FIFO Full
    can_->ILS = FDCAN_ILS_TCL | FDCAN_ILS_BOE;  //Set Line 1 for Transmit Complete Event Interrupt and Bus Off Interrupt
    // And Busoff error
    can_->TXBTIE = 0xFFFFFFFF;
    can_->ILE = 0x3;

    // If mode is Filtered then we finish the initialisation in configureFilter method
    // otherwise we finish here
    if (mode != FilteredMode) {
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        uint32_t while_start_ms = AP_HAL::millis();
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
            if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                return false;
            }
        }

        //initialised
        initialised_ = true;
    }
    return true;
}

void CANIface::clear_rx()
{
    CriticalSectionLocker lock;
    rx_queue_.clear();
}

void CANIface::setupMessageRam()
{
    uint32_t num_elements = 0;

    // Rx FIFO 0 start address and element count
    num_elements = MIN((FDCAN_NUM_RXFIFO0_SIZE/FDCAN_FRAME_BUFFER_SIZE), 64U);
    if (num_elements) {
        can_->RXF0C = (FDCANMessageRAMOffset_ << 2) | (num_elements << 16);
        MessageRam_.RxFIFO0SA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_elements*FDCAN_FRAME_BUFFER_SIZE;
    }

    // Tx FIFO/queue start address and element count
    num_elements = MIN((FDCAN_TX_FIFO_BUFFER_SIZE/FDCAN_FRAME_BUFFER_SIZE), 32U);
    if (num_elements) {
        can_->TXBC = (FDCANMessageRAMOffset_ << 2) | (num_elements << 24);
        MessageRam_.TxFIFOQSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_elements*FDCAN_FRAME_BUFFER_SIZE;
    }
    MessageRam_.EndAddress = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    if (MessageRam_.EndAddress > MESSAGE_RAM_END_ADDR) {
        //We are overflowing the limit of Allocated Message RAM
        AP_HAL::panic("CANFDIface: Message RAM Overflow!");
        return;
    }
}

void CANIface::handleTxCompleteInterrupt(const uint64_t timestamp_us)
{
    for (uint8_t i = 0; i < NumTxMailboxes; i++) {
        if ((can_->TXBTO & (1UL << i))) {

            if (!pending_tx_[i].pushed) {
                stats.tx_success++;
                pending_tx_[i].pushed = true;
            } else {
                continue;
            }

            if (pending_tx_[i].loopback && had_activity_) {
                CanRxItem rx_item;
                rx_item.frame = pending_tx_[i].frame;
                rx_item.timestamp_us = timestamp_us;
                rx_item.flags = AP_HAL::CANIface::Loopback;
                rx_queue_.push(rx_item);
            }
            if (event_handle_ != nullptr) {
                stats.num_events++;
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
                evt_src_.signalI(1 << self_index_);
#endif
            }
        }
    }
}

bool CANIface::readRxFIFO(uint8_t fifo_index)
{
    uint32_t *frame_ptr;
    uint32_t index;
    uint64_t timestamp_us = AP_HAL::micros64();
    if (fifo_index == 0) {
        //Check if RAM allocated to RX FIFO
        if ((can_->RXF0C & FDCAN_RXF0C_F0S) == 0) {
            return false;
        }
        //Register Message Lost as a hardware error
        if ((can_->RXF0S & FDCAN_RXF0S_RF0L) != 0) {
            stats.rx_errors++;
        }

        if ((can_->RXF0S & FDCAN_RXF0S_F0FL) == 0) {
            return false; //No More messages in FIFO
        } else {
            index = ((can_->RXF0S & FDCAN_RXF0S_F0GI) >> 8);
            frame_ptr = (uint32_t *)(MessageRam_.RxFIFO0SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));
        }
    } else if (fifo_index == 1) {
        //Check if RAM allocated to RX FIFO
        if ((can_->RXF1C & FDCAN_RXF1C_F1S) == 0) {
            return false;
        }
        //Register Message Lost as a hardware error
        if ((can_->RXF1S & FDCAN_RXF1S_RF1L) != 0) {
            stats.rx_errors++;
        }

        if ((can_->RXF1S & FDCAN_RXF1S_F1FL) == 0) {
            return false;
        } else {
            index = ((can_->RXF1S & FDCAN_RXF1S_F1GI) >> 8);
            frame_ptr = (uint32_t *)(MessageRam_.RxFIFO1SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));
        }
    } else {
        return false;
    }

    // Read the frame contents
    AP_HAL::CANFrame frame;
    uint32_t id = frame_ptr[0];
    if ((id & IDE) == 0) {
        //Standard ID
        frame.id = ((id & STID_MASK) >> 18) & AP_HAL::CANFrame::MaskStdID;
    } else {
        //Extended ID
        frame.id = (id & EXID_MASK) & AP_HAL::CANFrame::MaskExtID;
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }

    if ((id & RTR) != 0) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }
    frame.dlc = (frame_ptr[1] & DLC_MASK) >> 16;
    uint8_t *data = (uint8_t*)&frame_ptr[2];
    //We only handle Data Length of 8 Bytes for now
    for (uint8_t i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }

    //Acknowledge the FIFO entry we just read
    if (fifo_index == 0) {
        can_->RXF0A = index;
    } else if (fifo_index == 1) {
        can_->RXF1A = index;
    }

    /*
     * Store with timeout into the FIFO buffer
     */

    CanRxItem rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (rx_queue_.push(rx_item)) {
        stats.rx_received++;
    } else {
        stats.rx_overflow++;
    }
    return true;
}

void CANIface::handleRxInterrupt(uint8_t fifo_index)
{
    while (readRxFIFO(fifo_index)) {
        had_activity_ = true;
    }
    if (event_handle_ != nullptr) {
        stats.num_events++;
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
        evt_src_.signalI(1 << self_index_);
#endif
    }
}

/**
 * This method is used to count errors and abort transmission on error if necessary.
 * This functionality used to be implemented in the SCE interrupt handler, but that approach was
 * generating too much processing overhead, especially on disconnected interfaces.
 *
 * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
 */
void CANIface::pollErrorFlagsFromISR()
{
    const uint8_t cel = can_->ECR >> 16;

    if (cel != 0) {
        for (int i = 0; i < NumTxMailboxes; i++) {
            if (!pending_tx_[i].abort_on_error || pending_tx_[i].aborted) {
                continue;
            }
            if (((1 << pending_tx_[i].index) & can_->TXBRP)) {
                can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
                pending_tx_[i].aborted = true;
                stats.tx_abort++;
            }
        }
    }
}

void CANIface::pollErrorFlags()
{
    CriticalSectionLocker cs_locker;
    pollErrorFlagsFromISR();
}

bool CANIface::canAcceptNewTxFrame() const
{
    //Check if Tx FIFO is allocated
    if ((can_->TXBC & FDCAN_TXBC_TFQS) == 0) {
        return false;
    }
    if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        return false;    //we don't have free space
    }

    return true;
}

/**
 * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
 * May increase continuously if the interface is not connected to the bus.
 */
uint32_t CANIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return stats.num_busoff_err +
           stats.rx_errors +
           stats.rx_overflow +
           stats.tx_rejected +
           stats.tx_abort +
           stats.tx_timedout;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
ChibiOS::EventSource CANIface::evt_src_;
bool CANIface::set_event_handle(AP_HAL::EventHandle* handle)
{
    CriticalSectionLocker lock;
    event_handle_ = handle;
    event_handle_->set_source(&evt_src_);
    return event_handle_->register_event(1 << self_index_);
}
#endif

bool CANIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

void CANIface::clearErrors()
{
    if (_detected_bus_off) {
        //Try Recovering from BusOff
        //While in Bus off mode the CAN Peripheral is put
        //into INIT mode, when we ask Peripheral to get out
        //of INIT mode, the bit stream processor (BSP) synchronizes
        //itself to the data transfer on the CAN bus by
        //waiting for the occurrence of a sequence of 11 consecutive
        //recessive bits (Bus_Idle) before it can take part in bus
        //activities and start the message transfer
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        stats.num_busoff_err++;
        _detected_bus_off = false;
    }
}

void CANIface::discardTimedOutTxMailboxes(uint64_t current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        if (pending_tx_[i].aborted || !pending_tx_[i].setup) {
            continue;
        }
        if (((1 << pending_tx_[i].index) & can_->TXBRP) && pending_tx_[i].deadline < current_time) {
            can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
            pending_tx_[i].aborted = true;
            stats.tx_timedout++;
        }
    }
}

void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    write = false;
    read = !isRxBufferEmpty();
    if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame();
    }
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    const bool in_read = read;
    const bool in_write= write;
    uint64_t time = AP_HAL::micros();

    if (!read && !write) {
        //invalid request
        return false;
    }

    discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    pollErrorFlags();
    clearErrors();

    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }
    while (time < blocking_deadline) {
        if (event_handle_ == nullptr) {
            break;
        }
        event_handle_->wait(blocking_deadline - time); // Block until timeout expires or any iface updates
        checkAvailable(read, write, pending_tx);  // Check what we got
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        time = AP_HAL::micros();
    }
    return true; // Return value doesn't matter as long as it is non-negative
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
uint32_t CANIface::get_stats(char* data, uint32_t max_size)
{
    if (data == nullptr) {
        return 0;
    }
    CriticalSectionLocker lock;
    uint32_t ret = snprintf(data, max_size,
                            "tx_requests:    %lu\n"
                            "tx_rejected:    %lu\n"
                            "tx_success:     %lu\n"
                            "tx_timedout:    %lu\n"
                            "tx_abort:       %lu\n"
                            "rx_received:    %lu\n"
                            "rx_overflow:    %lu\n"
                            "rx_errors:      %lu\n"
                            "num_busoff_err: %lu\n"
                            "num_events:     %lu\n",
                            stats.tx_requests,
                            stats.tx_rejected,
                            stats.tx_success,
                            stats.tx_timedout,
                            stats.tx_abort,
                            stats.rx_received,
                            stats.rx_overflow,
                            stats.rx_errors,
                            stats.num_busoff_err,
                            stats.num_events);
    return ret;
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{

    CH_IRQ_HANDLER(FDCAN1_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN1_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(0, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN1_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN1_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(0, 1);
        CH_IRQ_EPILOGUE();
    }


# if HAL_NUM_CAN_IFACES > 1

    CH_IRQ_HANDLER(FDCAN2_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN2_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(1, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN2_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN2_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(1, 1);
        CH_IRQ_EPILOGUE();
    }

# endif

} // extern "C"

#endif //defined(STM32H7XX)

#endif //HAL_NUM_CAN_IFACES
