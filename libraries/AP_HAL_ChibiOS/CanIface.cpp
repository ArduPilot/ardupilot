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

#include <hal.h>
#include "AP_HAL_ChibiOS.h"

#if HAL_NUM_CAN_IFACES
#include <cassert>
#include <cstring>
#include <AP_Math/AP_Math.h>
# include <hal.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/ExpandingString.h>

# if !defined(STM32H7XX) && !defined(STM32G4)
#include "CANIface.h"

/* STM32F3's only CAN inteface does not have a number. */
#if defined(STM32F3XX)
#define RCC_APB1ENR_CAN1EN     RCC_APB1ENR_CANEN
#define RCC_APB1RSTR_CAN1RST   RCC_APB1RSTR_CANRST
#define CAN1_TX_IRQn           CAN_TX_IRQn
#define CAN1_RX0_IRQn          CAN_RX0_IRQn
#define CAN1_RX1_IRQn          CAN_RX1_IRQn
#define CAN1_TX_IRQ_Handler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQ_Handler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQ_Handler     STM32_CAN1_RX1_HANDLER
#else
#define CAN1_TX_IRQ_Handler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQ_Handler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQ_Handler     STM32_CAN1_RX1_HANDLER
#define CAN2_TX_IRQ_Handler      STM32_CAN2_TX_HANDLER
#define CAN2_RX0_IRQ_Handler     STM32_CAN2_RX0_HANDLER
#define CAN2_RX1_IRQ_Handler     STM32_CAN2_RX1_HANDLER
#endif // #if defined(STM32F3XX)

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
#define PERF_STATS(x) (x++)
#else
#define PERF_STATS(x)
#endif


extern AP_HAL::HAL& hal;

using namespace ChibiOS;

constexpr bxcan::CanType* const CANIface::Can[];
static ChibiOS::CANIface* can_ifaces[HAL_NUM_CAN_IFACES];

uint8_t CANIface::next_interface;

// mapping from logical interface to physical. First physical is 0, first logical is 0
static constexpr uint8_t can_interfaces[HAL_NUM_CAN_IFACES] = { HAL_CAN_INTERFACE_LIST };

// mapping from physical interface back to logical. First physical is 0, first logical is 0
static constexpr int8_t can_iface_to_idx[3] = { HAL_CAN_INTERFACE_REV_LIST };

static inline void handleTxInterrupt(uint8_t phys_index)
{
    const int8_t iface_index = can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleTxInterrupt(precise_time);
    }
}

static inline void handleRxInterrupt(uint8_t phys_index, uint8_t fifo_index)
{
    const int8_t iface_index = can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
    uint64_t precise_time = AP_HAL::micros64();
    if (precise_time > 0) {
        precise_time--;
    }
    if (can_ifaces[iface_index] != nullptr) {
        can_ifaces[iface_index]->handleRxInterrupt(fifo_index, precise_time);
    }
}

/*
 * CANIface
 */
const uint32_t CANIface::TSR_ABRQx[CANIface::NumTxMailboxes] = {
    bxcan::TSR_ABRQ0,
    bxcan::TSR_ABRQ1,
    bxcan::TSR_ABRQ2
};


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

// constructor suitable for array
CANIface::CANIface() :
    CANIface(next_interface++)
{}

bool CANIface::computeTimings(uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return false;
    }

    /*
     * Hardware configuration
     */
    const uint32_t pclk = STM32_PCLK1;

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

    Debug("Timings: quanta/bit: %d, sample point location: %.1f%%",
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
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -1;
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

    {
        CriticalSectionLocker lock;

        /*
         * Seeking for an empty slot
         */
        uint8_t txmailbox = 0xFF;
        if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0) {
            txmailbox = 0;
        } else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1) {
            txmailbox = 1;
        } else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2) {
            txmailbox = 2;
        } else {
            PERF_STATS(stats.tx_rejected);
            return 0;       // No transmission for you.
        }

        /*
         * Setting up the mailbox
         */
        bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
        if (frame.isExtended()) {
            mb.TIR = ((frame.id & AP_HAL::CANFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
        } else {
            mb.TIR = ((frame.id & AP_HAL::CANFrame::MaskStdID) << 21);
        }

        if (frame.isRemoteTransmissionRequest()) {
            mb.TIR |= bxcan::TIR_RTR;
        }

        mb.TDTR = frame.dlc;

        mb.TDHR = frame.data_32[1];
        mb.TDLR = frame.data_32[0];

        mb.TIR |= bxcan::TIR_TXRQ;  // Go.

        /*
         * Registering the pending transmission so we can track its deadline and loopback it as needed
         */
        CanTxItem& txi = pending_tx_[txmailbox];
        txi.deadline       = tx_deadline;
        txi.frame          = frame;
        txi.loopback       = (flags & Loopback) != 0;
        txi.abort_on_error = (flags & AbortOnError) != 0;
        // setup frame initial state
        txi.pushed         = false;
    }

    // also send on MAVCAN, but don't consider it an error if we can't send
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    {
        CriticalSectionLocker lock;
        CanRxItem rx_item;
        if (!rx_queue_.pop(rx_item)) {
            return 0;
        }
        out_frame    = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags    = rx_item.flags;
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

#if !defined(HAL_BOOTLOADER_BUILD)
bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
#if !defined(HAL_BUILD_AP_PERIPH)
    // only do filtering for AP_Periph
    can_->FMR &= ~bxcan::FMR_FINIT;
    return true;
#else
    if (mode_ != FilteredMode) {
        return false;
    }
    if (num_configs <= NumFilters && filter_configs != nullptr) {
        CriticalSectionLocker lock;

        can_->FMR |= bxcan::FMR_FINIT;

        // Slave (CAN2) gets half of the filters
        can_->FMR &= ~0x00003F00UL;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;

        can_->FFA1R = 0x0AAAAAAA; // FIFO's are interleaved between filters
        can_->FM1R = 0; // Identifier Mask mode
        can_->FS1R = 0x7ffffff; // Single 32-bit for all

        const uint8_t filter_start_index = (self_index_ == 0) ? 0 : NumFilters;

        if (num_configs == 0) {
            can_->FilterRegister[filter_start_index].FR1 = 0;
            can_->FilterRegister[filter_start_index].FR2 = 0;
            can_->FA1R = 1 << filter_start_index;
        } else {
            for (uint8_t i = 0; i < NumFilters; i++) {
                if (i < num_configs) {
                    uint32_t id   = 0;
                    uint32_t mask = 0;

                    const CanFilterConfig* const cfg = filter_configs + i;

                    if ((cfg->id & AP_HAL::CANFrame::FlagEFF) || !(cfg->mask & AP_HAL::CANFrame::FlagEFF)) {
                        id   = (cfg->id   & AP_HAL::CANFrame::MaskExtID) << 3;
                        mask = (cfg->mask & AP_HAL::CANFrame::MaskExtID) << 3;
                        id |= bxcan::RIR_IDE;
                    } else {
                        id   = (cfg->id   & AP_HAL::CANFrame::MaskStdID) << 21;  // Regular std frames, nothing fancy.
                        mask = (cfg->mask & AP_HAL::CANFrame::MaskStdID) << 21;  // Boring.
                    }

                    if (cfg->id & AP_HAL::CANFrame::FlagRTR) {
                        id |= bxcan::RIR_RTR;
                    }

                    if (cfg->mask & AP_HAL::CANFrame::FlagEFF) {
                        mask |= bxcan::RIR_IDE;
                    }

                    if (cfg->mask & AP_HAL::CANFrame::FlagRTR) {
                        mask |= bxcan::RIR_RTR;
                    }

                    can_->FilterRegister[filter_start_index + i].FR1 = id;
                    can_->FilterRegister[filter_start_index + i].FR2 = mask;

                    can_->FA1R |= (1 << (filter_start_index + i));
                } else {
                    can_->FA1R &= ~(1 << (filter_start_index + i));
                }
            }
        }

        can_->FMR &= ~bxcan::FMR_FINIT;

        return true;
    }

    return false;
#endif // AP_Periph
}
#endif

bool CANIface::waitMsrINakBitStateChange(bool target_state)
{
    const unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state) {
            return true;
        }
        chThdSleep(chTimeMS2I(1));
    }
    return false;
}

void CANIface::handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t timestamp_us)
{
    if (mailbox_index > NumTxMailboxes) {
        return;
    }

    had_activity_ = had_activity_ || txok;

    CanTxItem& txi = pending_tx_[mailbox_index];

    if (txi.loopback && txok && !txi.pushed) {
        CanRxItem rx_item;
        rx_item.frame = txi.frame;
        rx_item.timestamp_us = timestamp_us;
        rx_item.flags = AP_HAL::CANIface::Loopback;
        add_to_rx_queue(rx_item);
    }

    if (txok && !txi.pushed) {
        txi.pushed = true;
        PERF_STATS(stats.tx_success);
    }
}

void CANIface::handleTxInterrupt(const uint64_t utc_usec)
{
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & bxcan::TSR_RQCP0) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP1) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP2) {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }

#if CH_CFG_USE_EVENTS == TRUE
    if (event_handle_ != nullptr) {
        PERF_STATS(stats.num_events);
        evt_src_.signalI(1 << self_index_);
    }
#endif
    pollErrorFlagsFromISR();
}

void CANIface::handleRxInterrupt(uint8_t fifo_index, uint64_t timestamp_us)
{
    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0) {
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0) {
        PERF_STATS(stats.rx_errors);
    }

    /*
     * Read the frame contents
     */
    AP_HAL::CANFrame &frame = isr_rx_frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0) {
        frame.id = AP_HAL::CANFrame::MaskStdID & (rf.RIR >> 21);
    } else {
        frame.id = AP_HAL::CANFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= AP_HAL::CANFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0) {
        frame.id |= AP_HAL::CANFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uint8_t(0xFF & (rf.RDHR >> 24));

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    CanRxItem &rx_item = isr_rx_item;
    rx_item.frame = frame;
    rx_item.timestamp_us = timestamp_us;
    rx_item.flags = 0;
    if (add_to_rx_queue(rx_item)) {
        PERF_STATS(stats.rx_received);
    } else {
        PERF_STATS(stats.rx_overflow);
    }

    had_activity_ = true;

#if CH_CFG_USE_EVENTS == TRUE
    if (event_handle_ != nullptr) {
        PERF_STATS(stats.num_events);
        evt_src_.signalI(1 << self_index_);
    }
#endif
    pollErrorFlagsFromISR();
}

void CANIface::pollErrorFlagsFromISR()
{
    const uint8_t lec = uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0) {
#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
        stats.esr = can_->ESR; // Record error status
#endif
        can_->ESR = 0;

        // Serving abort requests
        for (int i = 0; i < NumTxMailboxes; i++) {
            CanTxItem& txi = pending_tx_[i];
            if (txi.aborted && txi.abort_on_error) {
                can_->TSR = TSR_ABRQx[i];
                txi.aborted = true;
                PERF_STATS(stats.tx_abort);
            }
        }
    }
}

void CANIface::discardTimedOutTxMailboxes(uint64_t current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        CanTxItem& txi = pending_tx_[i];
        if (txi.aborted || !txi.setup) {
            continue;
        }
        if (txi.deadline < current_time) {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            pending_tx_[i].aborted = true;
            PERF_STATS(stats.tx_timedout);
        }
    }
}

void CANIface::clear_rx()
{
    CriticalSectionLocker lock;
    rx_queue_.clear();
}

void CANIface::pollErrorFlags()
{
    CriticalSectionLocker cs_locker;
    pollErrorFlagsFromISR();
}

bool CANIface::canAcceptNewTxFrame(const AP_HAL::CANFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        static const uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        const uint32_t tme = can_->TSR & TME;

        if (tme == TME) {   // All TX mailboxes are free (as in freedom).
            return true;
        }

        if (tme == 0) {     // All TX mailboxes are busy transmitting.
            return false;
        }
    }

    /*
     * The second condition requires a critical section.
     */
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
        if (!(pending_tx_[mbx].pushed || pending_tx_[mbx].aborted) && !frame.priorityHigherThan(pending_tx_[mbx].frame)) {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
}

bool CANIface::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
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

#endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)

#if CH_CFG_USE_EVENTS == TRUE
ChibiOS::EventSource CANIface::evt_src_;
bool CANIface::set_event_handle(AP_HAL::EventHandle* handle)
{
    CriticalSectionLocker lock;
    event_handle_ = handle;
    event_handle_->set_source(&evt_src_);
    return event_handle_->register_event(1 << self_index_);
}

#endif // #if CH_CFG_USE_EVENTS == TRUE

void CANIface::checkAvailable(bool& read, bool& write, const AP_HAL::CANFrame* pending_tx) const
{
    write = false;
    read = !isRxBufferEmpty();

    if (pending_tx != nullptr) {
        write = canAcceptNewTxFrame(*pending_tx);
    }
}

bool CANIface::select(bool &read, bool &write,
                      const AP_HAL::CANFrame* pending_tx,
                      uint64_t blocking_deadline)
{
    const bool in_read = read;
    const bool in_write= write;
    uint64_t time = AP_HAL::micros64();

    if (!read && !write) {
        //invalid request
        return false;
    }

    discardTimedOutTxMailboxes(time);              // Check TX timeouts - this may release some TX slots
    pollErrorFlags();

    checkAvailable(read, write, pending_tx);          // Check if we already have some of the requested events
    if ((read && in_read) || (write && in_write)) {
        return true;
    }

#if CH_CFG_USE_EVENTS == TRUE
    // we don't support blocking select in AP_Periph and bootloader
    while (time < blocking_deadline) {
        if (event_handle_ == nullptr) {
            break;
        }
        event_handle_->wait(blocking_deadline - time); // Block until timeout expires or any iface updates
        checkAvailable(read, write, pending_tx);  // Check what we got
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        time = AP_HAL::micros64();
    }
#endif // #if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
    return true;
}

void CANIface::initOnce(bool enable_irq)
{
    /*
     * CAN1, CAN2
     */
    {
        CriticalSectionLocker lock;
        switch (can_interfaces[self_index_]) {
        case 0:
#if defined(RCC_APB1ENR1_CAN1EN)
            RCC->APB1ENR1 |=  RCC_APB1ENR1_CAN1EN;
            RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN1RST;
            RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN1RST;
#else
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
#endif
            break;
#if defined(RCC_APB1ENR1_CAN2EN)
        case 1:
            RCC->APB1ENR1  |=  RCC_APB1ENR1_CAN2EN;
            RCC->APB1RSTR1 |=  RCC_APB1RSTR1_CAN2RST;
            RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_CAN2RST;
            break;
#elif defined(RCC_APB1ENR_CAN2EN)
        case 1:
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
            break;
#endif
#ifdef RCC_APB1ENR_CAN3EN
        case 2:
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN3EN;
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN3RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN3RST;
            break;
#endif
        }
    }

    /*
     * IRQ
     */
    if (!irq_init_ && enable_irq) {
        CriticalSectionLocker lock;
        switch (can_interfaces[self_index_]) {
        case 0:
#ifdef HAL_CAN_IFACE1_ENABLE
            nvicEnableVector(CAN1_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN1_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN1_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        case 1:
#ifdef HAL_CAN_IFACE2_ENABLE
            nvicEnableVector(CAN2_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN2_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN2_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        case 2:
#ifdef HAL_CAN_IFACE3_ENABLE
            nvicEnableVector(CAN3_TX_IRQn,  CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN3_RX0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(CAN3_RX1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
#endif
            break;
        }
        irq_init_ = true;
    }
}

bool CANIface::init(const uint32_t bitrate, const CANIface::OperatingMode mode)
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

    if (can_ifaces[0] == nullptr) {
        can_ifaces[0] = new CANIface(0);
        Debug("Failed to allocate CAN iface 0");
        if (can_ifaces[0] == nullptr) {
            return false;
        }
    }
    if (self_index_ == 1 && !can_ifaces[0]->is_initialized()) {
        Debug("Iface 0 is not initialized yet but we need it for Iface 1, trying to init it");
        Debug("Enabling CAN iface 0");
        can_ifaces[0]->initOnce(false);
        Debug("Initing iface 0...");
        if (!can_ifaces[0]->init(bitrate, mode)) {
            Debug("Iface 0 init failed");
            return false;
        }

        Debug("Enabling CAN iface");
    }
    initOnce(true);
    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        CriticalSectionLocker lock;

        can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
        can_->MCR |= bxcan::MCR_INRQ;   // Request init

        can_->IER = 0;                  // Disable interrupts while initialization is in progress
    }

    if (!waitMsrINakBitStateChange(true)) {
        Debug("MSR INAK not set");
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }

    /*
     * Object state - interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();

    for (uint32_t i=0; i < NumTxMailboxes; i++) {
        pending_tx_[i] = CanTxItem();
    }
    had_activity_ = false;

    /*
     * CAN timings for this bitrate
     */
    Timings timings;
    if (!computeTimings(bitrate, timings)) {
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }
    Debug("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    /*
     * Hardware initialization (the hardware has already confirmed initialization mode, see above)
     */
    can_->MCR = bxcan::MCR_ABOM | bxcan::MCR_AWUM | bxcan::MCR_INRQ;  // RM page 648

    can_->BTR = ((timings.sjw & 3U)  << 24) |
                ((timings.bs1 & 15U) << 16) |
                ((timings.bs2 & 7U)  << 20) |
                (timings.prescaler & 1023U) |
                ((mode == SilentMode) ? bxcan::BTR_SILM : 0);

    can_->IER = bxcan::IER_TMEIE |   // TX mailbox empty
                bxcan::IER_FMPIE0 |  // RX FIFO 0 is not empty
                bxcan::IER_FMPIE1;   // RX FIFO 1 is not empty

    can_->MCR &= ~bxcan::MCR_INRQ;   // Leave init mode

    if (!waitMsrINakBitStateChange(false)) {
        Debug("MSR INAK not cleared");
        can_->MCR = bxcan::MCR_RESET;
        return false;
    }

    /*
     * Default filter configuration
     */
    if (self_index_ == 0) {
        can_->FMR |= bxcan::FMR_FINIT;

        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

        can_->FFA1R = 0;                           // All assigned to FIFO0 by default
        can_->FM1R = 0;                            // Indentifier Mask mode

#if HAL_NUM_CAN_IFACES > 1
        can_->FS1R = 0x7ffffff;                    // Single 32-bit for all
        can_->FilterRegister[0].FR1 = 0;          // CAN1 accepts everything
        can_->FilterRegister[0].FR2 = 0;
        can_->FilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
        can_->FilterRegister[NumFilters].FR2 = 0;
        can_->FA1R = 1 | (1 << NumFilters);        // One filter per each iface
#else
        can_->FS1R = 0x1fff;
        can_->FilterRegister[0].FR1 = 0;
        can_->FilterRegister[0].FR2 = 0;
        can_->FA1R = 1;
#endif

        can_->FMR &= ~bxcan::FMR_FINIT;
    }
    initialised_ = true;

    return true;
}

#if !defined(HAL_BUILD_AP_PERIPH) && !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    CriticalSectionLocker lock;
    str.printf("tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "num_busoff_err: %lu\n"
               "num_events:     %lu\n"
               "ESR:            %lx\n",
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_success,
               stats.tx_timedout,
               stats.tx_abort,
               stats.rx_received,
               stats.rx_overflow,
               stats.rx_errors,
               stats.num_busoff_err,
               stats.num_events,
               stats.esr);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE1_ENABLE
    // CAN1
    CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_TX_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleTxInterrupt(0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_RX0_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(0, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN1_RX1_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(0, 1);
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE2_ENABLE
    // CAN2
    CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_TX_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleTxInterrupt(1);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_RX0_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(1, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN2_RX1_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(1, 1);
        CH_IRQ_EPILOGUE();
    }
#endif

#ifdef HAL_CAN_IFACE3_ENABLE
    // CAN3
    CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_TX_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleTxInterrupt(2);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_RX0_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(2, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler);
    CH_IRQ_HANDLER(CAN3_RX1_IRQ_Handler)
    {
        CH_IRQ_PROLOGUE();
        handleRxInterrupt(2, 1);
        CH_IRQ_EPILOGUE();
    }
#endif
    
} // extern "C"

#endif //!defined(STM32H7XX)

#endif //HAL_NUM_CAN_IFACES
