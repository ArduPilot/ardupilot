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

#if HAL_WITH_UAVCAN
#include <cassert>
#include <cstring>
#include "CANClock.h"
#include "CANInternal.h"
#include "CANSerialRouter.h"
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
#include <AP_Math/AP_Math.h>
# include <hal.h>

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

extern const AP_HAL::HAL& hal;

namespace ChibiOS_CAN
{
namespace
{

CanIface* ifaces[UAVCAN_STM32_NUM_IFACES] = {
    UAVCAN_NULLPTR
#if UAVCAN_STM32_NUM_IFACES > 1
    , UAVCAN_NULLPTR
#endif
};

inline void handleInterrupt(uavcan::uint8_t iface_index, uavcan::uint8_t line_index)
{
    UAVCAN_ASSERT(iface_index < UAVCAN_STM32_NUM_IFACES);
    if (ifaces[iface_index] == UAVCAN_NULLPTR) {
        //Just reset all the interrupts and return
        fdcan::Can[iface_index]->IR = FDCAN_IR_RF0N;
        fdcan::Can[iface_index]->IR = FDCAN_IR_RF1N;
        fdcan::Can[iface_index]->IR = FDCAN_IR_TEFN;
        UAVCAN_ASSERT(0);
        return;
    }
    if (line_index == 0) {
        if ((ifaces[iface_index]->can_reg()->IR & FDCAN_IR_RF0N) ||
           (ifaces[iface_index]->can_reg()->IR & FDCAN_IR_RF0F)) {
            ifaces[iface_index]->can_reg()->IR = FDCAN_IR_RF0N | FDCAN_IR_RF0F;
            ifaces[iface_index]->handleRxInterrupt(0);
        }
        if ((ifaces[iface_index]->can_reg()->IR & FDCAN_IR_RF1N) ||
           (ifaces[iface_index]->can_reg()->IR & FDCAN_IR_RF1F)) {
            ifaces[iface_index]->can_reg()->IR = FDCAN_IR_RF1N | FDCAN_IR_RF1F;
            ifaces[iface_index]->handleRxInterrupt(1);
        }
    } else {
        if (ifaces[iface_index]->can_reg()->IR & FDCAN_IR_TC) {
            ifaces[iface_index]->can_reg()->IR = FDCAN_IR_TC;
            uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
            if (utc_usec > 0) {
                utc_usec--;
            }
            ifaces[iface_index]->handleTxCompleteInterrupt(utc_usec);
        }
    }
    ifaces[iface_index]->pollErrorFlagsFromISR();
}

} // namespace

uint32_t CanIface::FDCANMessageRAMOffset_ = 0;
#if AP_UAVCAN_SLCAN_ENABLED
SLCANRouter CanIface::_slcan_router;
#endif

CanIface::CanIface(fdcan::CanType* can, BusEvent& update_event, uavcan::uint8_t self_index,
            CanRxItem* rx_queue_buffer, uavcan::uint8_t rx_queue_capacity)
    : rx_queue_(rx_queue_buffer, rx_queue_capacity)
    , can_(can)
    , error_cnt_(0)
    , served_aborts_cnt_(0)
    , update_event_(update_event)
    , peak_tx_mailbox_index_(0)
    , self_index_(self_index)
    , had_activity_(false)
{
    UAVCAN_ASSERT(self_index_ < UAVCAN_STM32_NUM_IFACES);
}

/*
 * CanIface::RxQueue
 */
void CanIface::RxQueue::registerOverflow()
{
    if (overflow_cnt_ < 0xFFFFFFFF) {
        overflow_cnt_++;
    }
}

void CanIface::RxQueue::push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags)
{
    buf_[in_].frame    = frame;
    buf_[in_].utc_usec = utc_usec;
    buf_[in_].flags    = flags;
    in_++;
    if (in_ >= capacity_) {
        in_ = 0;
    }
    len_++;
    if (len_ > capacity_) {
        len_ = capacity_;
        registerOverflow();
        out_++;
        if (out_ >= capacity_) {
            out_ = 0;
        }
    }
}

void CanIface::RxQueue::pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags)
{
    if (len_ > 0) {
        out_frame    = buf_[out_].frame;
        out_utc_usec = buf_[out_].utc_usec;
        out_flags    = buf_[out_].flags;
        out_++;
        if (out_ >= capacity_) {
            out_ = 0;
        }
        len_--;
    } else {
        UAVCAN_ASSERT(0);
    }
}

void CanIface::RxQueue::reset()
{
    in_ = 0;
    out_ = 0;
    len_ = 0;
    overflow_cnt_ = 0;
}

int CanIface::computeTimings(const uavcan::uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return -ErrInvalidBitRate;
    }

    /*
     * Hardware configuration
     */
    const uavcan::uint32_t pclk = STM32_PLL1_Q_CK;

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

    UAVCAN_ASSERT(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

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
    const uavcan::uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uavcan::uint8_t bs1_bs2_sum = uavcan::uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return -ErrInvalidBitRate;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uavcan::uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return -ErrInvalidBitRate;              // No solution
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
        uavcan::uint8_t bs1;
        uavcan::uint8_t bs2;
        uavcan::uint16_t sample_point_permill;

        BsPair() :
            bs1(0),
            bs2(0),
            sample_point_permill(0)
        { }

        BsPair(uavcan::uint8_t bs1_bs2_sum, uavcan::uint8_t arg_bs1) :
            bs1(arg_bs1),
            bs2(uavcan::uint8_t(bs1_bs2_sum - bs1)),
            sample_point_permill(uavcan::uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {
            UAVCAN_ASSERT(bs1_bs2_sum > arg_bs1);
        }

        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uavcan::uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uavcan::uint8_t((7 * bs1_bs2_sum - 1) / 8));
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
        UAVCAN_ASSERT(0);
        return -ErrLogic;
    }

    UAVCAN_STM32_LOG("Timings: quanta/bit: %d, sample point location: %.1f%%",
                     int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

    out_timings.prescaler = uavcan::uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                        // Which means one
    out_timings.bs1 = uavcan::uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uavcan::uint8_t(solution.bs2 - 1);
    return 0;
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                               uavcan::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -ErrUnsupportedFrame;
    }

    /*
     * Normally we should perform the same check as in @ref canAcceptNewTxFrame(), because
     * it is possible that the highest-priority frame between select() and send() could have been
     * replaced with a lower priority one due to TX timeout. But we don't do this check because:
     *
     *  - It is a highly unlikely scenario.
     *
     *  - Frames do not timeout on a properly functioning bus. Since frames do not timeout, the new
     *    frame can only have higher priority, which doesn't break the logic.
     *
     *  - If high-priority frames are timing out in the TX queue, there's probably a lot of other
     *    issues to take care of before this one becomes relevant.
     *
     *  - It takes CPU time. Not just CPU time, but critical section time, which is expensive.
     */
    CriticalSectionLocker lock;

    /*
     * Seeking for an empty slot
     */
    uavcan::uint8_t index;

    if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
        return false;    //we don't have free space
    }
    index = ((can_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);

    // Copy Frame to RAM
    // Calculate Tx element address
    uint32_t* buffer = (uint32_t *)(MessageRam_.TxFIFOQSA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));

    //Setup Frame ID
    if (frame.isExtended()) {
        buffer[0] = (fdcan::IDE | frame.id);
    } else {
        buffer[0] = (frame.id << 18);
    }
    if (frame.isRemoteTransmissionRequest()) {
        buffer[0] |= fdcan::RTR;
    }
    //Write Data Length Code, and Message Marker
    buffer[1] =  frame.dlc << 16 | index << 24;

    // Write Frame to the message RAM
    buffer[2] = (uavcan::uint32_t(frame.data[3]) << 24) |
                (uavcan::uint32_t(frame.data[2]) << 16) |
                (uavcan::uint32_t(frame.data[1]) << 8)  |
                (uavcan::uint32_t(frame.data[0]) << 0);
    buffer[3] = (uavcan::uint32_t(frame.data[7]) << 24) |
                (uavcan::uint32_t(frame.data[6]) << 16) |
                (uavcan::uint32_t(frame.data[5]) << 8)  |
                (uavcan::uint32_t(frame.data[4]) << 0);
    //Set Add Request
    can_->TXBAR = (1 << index);

    //Registering the pending transmission so we can track its deadline and loopback it as needed
    pending_tx_[index].deadline       = tx_deadline;
    pending_tx_[index].frame          = frame;
    pending_tx_[index].loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
    pending_tx_[index].abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    pending_tx_[index].index          = index;
    return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                  uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic();  // High precision is not required for monotonic timestamps
    uavcan::uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.getLength() == 0) {
            return 0;
        }
        rx_queue_.pop(out_frame, utc_usec, out_flags);
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

uavcan::int16_t CanIface::configureFilters(const uavcan::CanFilterConfig* filter_configs,
        uavcan::uint16_t num_configs)
{
    uint32_t num_extid = 0, num_stdid = 0;
    uint32_t total_available_list_size = MAX_FILTER_LIST_SIZE;
    uint32_t* filter_ptr;
    //count number of frames of each type
    for (uint8_t i = 0; i < num_configs; i++) {
        const uavcan::CanFilterConfig* const cfg = filter_configs + i;
        if ((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF)) {
            num_extid++;
        } else {
            num_stdid++;
        }
    }

    CriticalSectionLocker lock;
    can_->CCCR |= FDCAN_CCCR_INIT; // Request init
    while ((can_->CCCR & FDCAN_CCCR_INIT) == 0) {}
    can_->CCCR |= FDCAN_CCCR_CCE; //Enable Config change

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
        return -ErrFilterNumConfigs;
    }

    if (num_stdid) {
        num_stdid = 0;  //reset list count
        filter_ptr = (uint32_t*)MessageRam_.StandardFilterSA;
        //Run through the filter list and setup standard id filter list
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const uavcan::CanFilterConfig* const cfg = filter_configs + i;
            if (!((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF))) {
                id   = (cfg->id   & uavcan::CanFrame::MaskStdID);  // Regular std frames, nothing fancy.
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
        can_->GFC = (0x3U << 2); // Reject non matching Extended frames
    } else {    //The List is too big, return fail
        can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
        return -ErrFilterNumConfigs;
    }

    if (num_extid) {
        num_extid = 0;
        filter_ptr = (uint32_t*)MessageRam_.ExtendedFilterSA;
        //Run through the filter list and setup extended id filter list
        for (uint8_t i = 0; i < num_configs; i++) {
            uint32_t id = 0;
            uint32_t mask = 0;
            const uavcan::CanFilterConfig* const cfg = filter_configs + i;
            if ((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF)) {
                id   = (cfg->id   & uavcan::CanFrame::MaskExtID);
                mask = (cfg->mask & uavcan::CanFrame::MaskExtID);
                filter_ptr[num_extid*2]       = 0x1U << 29 | id; // Classic CAN Filter
                filter_ptr[num_extid*2 + 1]   = 0x2U << 30 | mask; //Store in Rx FIFO0 if filter matches
                num_extid++;
            }
        }
    }

    MessageRam_.EndAddress = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    if (MessageRam_.EndAddress > MESSAGE_RAM_END_ADDR) {
        //We are overflowing the limit of Allocated Message RAM
        AP_HAL::panic("CANFDIface: Message RAM Overflow!");
    }

    can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
    return 0;
}

uavcan::uint16_t CanIface::getNumFilters() const
{
    return MAX_FILTER_LIST_SIZE;
}

int CanIface::init(const uavcan::uint32_t bitrate, const OperatingMode mode)
{
    // Setup FDCAN for configuration mode and disable all interrupts
    {
        CriticalSectionLocker lock;

        can_->CCCR &= ~FDCAN_CCCR_CSR; // Exit sleep mode
        while ((can_->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) {} //Wait for wake up ack
        can_->CCCR |= FDCAN_CCCR_INIT; // Request init
        while ((can_->CCCR & FDCAN_CCCR_INIT) == 0) {}
        can_->CCCR |= FDCAN_CCCR_CCE; //Enable Config change
        can_->IE = 0;                  // Disable interrupts while initialization is in progress
    }

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.reset();
    error_cnt_ = 0;
    served_aborts_cnt_ = 0;
    uavcan::fill_n(pending_tx_, NumTxMailboxes, TxItem());
    peak_tx_mailbox_index_ = 0;
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    const int timings_res = computeTimings(bitrate, timings);
    if (timings_res < 0) {
        can_->CCCR &= ~FDCAN_CCCR_INIT;
        return timings_res;
    }
    UAVCAN_STM32_LOG("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
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
    //Clear all Interrupts
    can_->IR = 0x3FFFFFFF;
    //Enable Interrupts
    can_->IE =  FDCAN_IE_TCE |  // Transmit Complete interrupt enable
                FDCAN_IE_RF0NE |  // RX FIFO 0 new message
                FDCAN_IE_RF0FE |  // Rx FIFO 1 FIFO Full
                FDCAN_IE_RF1NE |  // RX FIFO 1 new message
                FDCAN_IE_RF1FE;   // Rx FIFO 1 FIFO Full
    can_->ILS = FDCAN_ILS_TCL;  //Set Line 1 for Transmit Complete Event Interrupt
    can_->TXBTIE = 0xFFFFFFFF;
    can_->ILE = 0x3;

    //Leave Init
    can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
    return 0;
}

void CanIface::setupMessageRam()
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
        can_->TXBC |= 1U << 30; //Set Queue mode
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

void CanIface::handleTxCompleteInterrupt(const uavcan::uint64_t utc_usec)
{
    for (uint8_t i = 0; i < NumTxMailboxes; i++) {
        if ((can_->TXBTO & (1UL << i))) {
            if (pending_tx_[i].loopback && had_activity_) {
                rx_queue_.push(pending_tx_[i].frame, utc_usec, uavcan::CanIOFlagLoopback);
            }
        }
    }
}

bool CanIface::readRxFIFO(uavcan::uint8_t fifo_index)
{
    UAVCAN_ASSERT(fifo_index < 2);
    uint32_t *frame_ptr;
    uint32_t index;
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (fifo_index == 0) {
        //Check if RAM allocated to RX FIFO
        if ((can_->RXF0C & FDCAN_RXF0C_F0S) == 0) {
            UAVCAN_ASSERT(0);
            return false;
        }
        //Register Message Lost as a hardware error
        if ((can_->RXF0S & FDCAN_RXF0S_RF0L) != 0) {
            error_cnt_++;
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
            UAVCAN_ASSERT(0);
            return false;
        }
        //Register Message Lost as a hardware error
        if ((can_->RXF1S & FDCAN_RXF1S_RF1L) != 0) {
            error_cnt_++;
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
    uavcan::CanFrame frame;
    uint32_t id = frame_ptr[0];
    if ((id & fdcan::IDE) == 0) {
        //Standard ID
        frame.id = ((id & fdcan::STID_MASK) >> 18) & uavcan::CanFrame::MaskStdID;
    } else {
        //Extended ID
        frame.id = (id & fdcan::EXID_MASK) & uavcan::CanFrame::MaskExtID;
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((id & fdcan::RTR) != 0) {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }
    frame.dlc = (frame_ptr[1] & fdcan::DLC_MASK) >> 16;
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
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, utc_usec, 0);
#if AP_UAVCAN_SLCAN_ENABLED
    _slcan_router.route_frame_to_slcan(this, frame, utc_usec);
#endif
    return true;
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index)
{
    while (readRxFIFO(fifo_index)) {
        had_activity_ = true;
    }
    update_event_.signalFromInterrupt();
}

void CanIface::pollErrorFlagsFromISR()
{
    const uavcan::uint8_t cel = can_->ECR >> 16;

    if (cel != 0) {
        for (int i = 0; i < NumTxMailboxes; i++) {
            if (!pending_tx_[i].abort_on_error) {
                continue;
            }
            if (((1 << pending_tx_[i].index) & can_->TXBRP)) {
                can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
                error_cnt_++;
                served_aborts_cnt_++;
            }
        }
    }
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        if (((1 << pending_tx_[i].index) & can_->TXBRP) && pending_tx_[i].deadline < current_time) {
            can_->TXBCR = 1 << pending_tx_[i].index;  // Goodnight sweet transmission
            error_cnt_++;
        }
    }
}

bool CanIface::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
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

bool CanIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength() == 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
    CriticalSectionLocker lock;
    return error_cnt_ + rx_queue_.getOverflowCount();
}

unsigned CanIface::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.getLength();
}

bool CanIface::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

/*
 * CanDriver
 */
uavcan::CanSelectMasks CanDriver::makeSelectMasks(const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces]) const
{
    uavcan::CanSelectMasks msk;

    for (uavcan::uint8_t i = 0; i < num_ifaces_; i++) {
        CanIface* iface = ifaces[if_int_to_gl_index_[i]];
        msk.read  |= (iface->isRxBufferEmpty() ? 0 : 1) << i;

        if (pending_tx[i] != UAVCAN_NULLPTR) {
            msk.write |= (iface->canAcceptNewTxFrame(*pending_tx[i]) ? 1 : 0) << i;
        }
    }

    return msk;
}

bool CanDriver::hasReadableInterfaces() const
{
    for (uavcan::uint8_t i = 0; i < num_ifaces_; i++) {
        if (!ifaces[if_int_to_gl_index_[i]]->isRxBufferEmpty()) {
            return true;
        }
    }

    return false;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks& inout_masks,
                                  const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
                                  const uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = clock::getMonotonic();

    for (uavcan::uint8_t i = 0; i < num_ifaces_; i++) {
        CanIface* iface = ifaces[if_int_to_gl_index_[i]];
        iface->discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
        {
            CriticalSectionLocker cs_locker;
            iface->pollErrorFlagsFromISR();
        }
    }

    inout_masks = makeSelectMasks(pending_tx);          // Check if we already have some of the requested events
    if ((inout_masks.read  & in_masks.read)  != 0 ||
        (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }

    (void)update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set
    return 1;                                   // Return value doesn't matter as long as it is non-negative
}

void CanDriver::initOnce()
{
    {
        CriticalSectionLocker lock;
        RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
        RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
        RCC->APB1HENR  |= RCC_APB1HENR_FDCANEN;
    }

    /*
     * IRQ
     */
    {
        CriticalSectionLocker lock;
        nvicEnableVector(FDCAN1_IT0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(FDCAN1_IT1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# if UAVCAN_STM32_NUM_IFACES > 1
        nvicEnableVector(FDCAN2_IT0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(FDCAN2_IT1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# endif
    }
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode)
{
    int res = 0;

    UAVCAN_STM32_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));

    static bool initialized_once = false;
    if (!initialized_once) {
        initialized_once = true;
        UAVCAN_STM32_LOG("First initialization");
        initOnce();
    }

    /*
     * CAN1
     */
    UAVCAN_STM32_LOG("Initing iface 0...");
    ifaces[0] = &if0_;                          // This link must be initialized first,
    res = if0_.init(bitrate, mode);             // otherwise an IRQ may fire while the interface is not linked yet;
    if (res < 0) {                              // a typical race condition.
        UAVCAN_STM32_LOG("Iface 0 init failed %i", res);
        ifaces[0] = UAVCAN_NULLPTR;
        goto fail;
    }

    /*
     * CAN2
     */
#if UAVCAN_STM32_NUM_IFACES > 1
    UAVCAN_STM32_LOG("Initing iface 1...");
    ifaces[1] = &if1_;                          // Same thing here.
    res = if1_.init(bitrate, mode);
    if (res < 0) {
        UAVCAN_STM32_LOG("Iface 1 init failed %i", res);
        ifaces[1] = UAVCAN_NULLPTR;
        goto fail;
    }
#endif

    UAVCAN_STM32_LOG("CAN drv init OK");
    UAVCAN_ASSERT(res >= 0);
    return res;

fail:
    UAVCAN_STM32_LOG("CAN drv init failed %i", res);
    UAVCAN_ASSERT(res < 0);
    return res;
}

bool CanDriver::clock_init_ = false;
void CanDriver::initOnce(uavcan::uint8_t can_number, bool enable_irqs)
{
    //Only do it once
    //Doing it second time will reset the previously initialised bus
    if (!clock_init_) {
        CriticalSectionLocker lock;
        RCC->APB1HENR  |= RCC_APB1HENR_FDCANEN;
        RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
        RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
        clock_init_ = true;
    }

    if (!enable_irqs) {
        return;
    }
    /*
     * IRQ
     */
    {
        CriticalSectionLocker lock;
        if (can_number == 0) {
            nvicEnableVector(FDCAN1_IT0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(FDCAN1_IT1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        }
# if UAVCAN_STM32_NUM_IFACES > 1
        else if (can_number == 1) {
            nvicEnableVector(FDCAN2_IT0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(FDCAN2_IT1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        }
# endif
    }
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode, uavcan::uint8_t can_number)
{
    int res = 0;

    UAVCAN_STM32_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    if (can_number > UAVCAN_STM32_NUM_IFACES) {
        res = -1;
        goto fail;
    }
    static bool initialized_once[UAVCAN_STM32_NUM_IFACES] = {false};

    if (!initialized_once[can_number]) {
        initialized_once[can_number] = true;
        initialized_by_me_[can_number] = true;

        if (can_number == 1 && !initialized_once[0]) {
            UAVCAN_STM32_LOG("Iface 0 is not initialized yet but we need it for Iface 1, trying to init it");
            UAVCAN_STM32_LOG("Enabling CAN iface 0");
            initOnce(0, false);
            UAVCAN_STM32_LOG("Initing iface 0...");
            res = if0_.init(bitrate, mode);

            if (res < 0) {
                UAVCAN_STM32_LOG("Iface 0 init failed %i", res);
                goto fail;
            }
        }

        UAVCAN_STM32_LOG("Enabling CAN iface %d", can_number);
        initOnce(can_number, true);
    } else if (!initialized_by_me_[can_number]) {
        UAVCAN_STM32_LOG("CAN iface %d initialized in another CANDriver!", can_number);
        res = -2;
        goto fail;
    }

    if (can_number == 0) {
        /*
        * CAN1
        */
        UAVCAN_STM32_LOG("Initing iface 0...");
        ifaces[0] = &if0_;                          // This link must be initialized first,
        res = if0_.init(bitrate, mode);             // otherwise an IRQ may fire while the interface is not linked yet;
        if (res < 0) {                              // a typical race condition.
            UAVCAN_STM32_LOG("Iface 0 init failed %i", res);
            ifaces[0] = UAVCAN_NULLPTR;
            goto fail;
        }
    } else if (can_number == 1) {
        /*
        * CAN2
        */
#if UAVCAN_STM32_NUM_IFACES > 1
        UAVCAN_STM32_LOG("Initing iface 1...");
        ifaces[1] = &if1_;                          // Same thing here.
        res = if1_.init(bitrate, mode);
        if (res < 0) {
            UAVCAN_STM32_LOG("Iface 1 init failed %i", res);
            ifaces[1] = UAVCAN_NULLPTR;
            goto fail;
        }
#endif
    }

    if_int_to_gl_index_[num_ifaces_++] = can_number;

    UAVCAN_STM32_LOG("CAN drv init OK");
    UAVCAN_ASSERT(res >= 0);
    return res;

fail:
    UAVCAN_STM32_LOG("CAN drv init failed %i", res);
    UAVCAN_ASSERT(res < 0);
    return res;
}

CanIface* CanDriver::getIface(uavcan::uint8_t iface_index)
{
    if (iface_index < num_ifaces_) {
        return ifaces[if_int_to_gl_index_[iface_index]];
    }
    return UAVCAN_NULLPTR;
}

bool CanDriver::hadActivity()
{
    for (uavcan::uint8_t i = 0; i < num_ifaces_; i++) {
        if (ifaces[if_int_to_gl_index_[i]]->hadActivity()) {
            return true;
        }
    }

    return false;
}

} // namespace uavcan_stm32

/*
 * Interrupt handlers
 */
extern "C"
{

    UAVCAN_STM32_IRQ_HANDLER(FDCAN1_IT0_IRQHandler);
    UAVCAN_STM32_IRQ_HANDLER(FDCAN1_IT0_IRQHandler)
    {
        UAVCAN_STM32_IRQ_PROLOGUE();
        ChibiOS_CAN::handleInterrupt(0, 0);
        UAVCAN_STM32_IRQ_EPILOGUE();
    }

    UAVCAN_STM32_IRQ_HANDLER(FDCAN1_IT1_IRQHandler);
    UAVCAN_STM32_IRQ_HANDLER(FDCAN1_IT1_IRQHandler)
    {
        UAVCAN_STM32_IRQ_PROLOGUE();
        ChibiOS_CAN::handleInterrupt(0, 1);
        UAVCAN_STM32_IRQ_EPILOGUE();
    }


# if UAVCAN_STM32_NUM_IFACES > 1

    UAVCAN_STM32_IRQ_HANDLER(FDCAN2_IT0_IRQHandler);
    UAVCAN_STM32_IRQ_HANDLER(FDCAN2_IT0_IRQHandler)
    {
        UAVCAN_STM32_IRQ_PROLOGUE();
        ChibiOS_CAN::handleInterrupt(1, 0);
        UAVCAN_STM32_IRQ_EPILOGUE();
    }

    UAVCAN_STM32_IRQ_HANDLER(FDCAN2_IT1_IRQHandler);
    UAVCAN_STM32_IRQ_HANDLER(FDCAN2_IT1_IRQHandler)
    {
        UAVCAN_STM32_IRQ_PROLOGUE();
        ChibiOS_CAN::handleInterrupt(1, 1);
        UAVCAN_STM32_IRQ_EPILOGUE();
    }

# endif

} // extern "C"

#endif //defined(STM32H7XX)

#endif //HAL_WITH_UAVCAN
