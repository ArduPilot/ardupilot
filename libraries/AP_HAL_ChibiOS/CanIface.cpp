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
#include "CANIface.h"
#include "CANClock.h"
#include "CANInternal.h"
#include "CANSerialRouter.h"
#include <AP_UAVCAN/AP_UAVCAN_SLCAN.h>
# include <hal.h>

#if CH_KERNEL_MAJOR == 2
# if !(defined(STM32F10X_CL) || defined(STM32F2XX) || defined(STM32F3XX)  || defined(STM32F4XX))
// IRQ numbers
#  define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#  define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
// IRQ vectors
#  if !defined(CAN1_RX0_IRQHandler) || !defined(CAN1_TX_IRQHandler)
#   define CAN1_TX_IRQHandler   USB_HP_CAN1_TX_IRQHandler
#   define CAN1_RX0_IRQHandler  USB_LP_CAN1_RX0_IRQHandler
#  endif
# endif
#endif

#if (CH_KERNEL_MAJOR == 3 || CH_KERNEL_MAJOR == 4 || CH_KERNEL_MAJOR == 5 || CH_KERNEL_MAJOR == 6)
#define CAN1_TX_IRQHandler      STM32_CAN1_TX_HANDLER
#define CAN1_RX0_IRQHandler     STM32_CAN1_RX0_HANDLER
#define CAN1_RX1_IRQHandler     STM32_CAN1_RX1_HANDLER
#define CAN2_TX_IRQHandler      STM32_CAN2_TX_HANDLER
#define CAN2_RX0_IRQHandler     STM32_CAN2_RX0_HANDLER
#define CAN2_RX1_IRQHandler     STM32_CAN2_RX1_HANDLER
#endif


/* STM32F3's only CAN inteface does not have a number. */
#if defined(STM32F3XX)
#define RCC_APB1ENR_CAN1EN     RCC_APB1ENR_CANEN
#define RCC_APB1RSTR_CAN1RST   RCC_APB1RSTR_CANRST
#define CAN1_TX_IRQn           CAN_TX_IRQn
#define CAN1_RX0_IRQn          CAN_RX0_IRQn
#define CAN1_RX1_IRQn          CAN_RX1_IRQn
#define CAN1_TX_IRQHandler     CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler    CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler    CAN_RX1_IRQHandler
#endif


namespace ChibiOS_CAN
{
namespace
{

CanIface* ifaces[UAVCAN_STM32_NUM_IFACES] =
{
    UAVCAN_NULLPTR
#if UAVCAN_STM32_NUM_IFACES > 1
    , UAVCAN_NULLPTR
#endif
};

inline void handleTxInterrupt(uavcan::uint8_t iface_index)
{
    UAVCAN_ASSERT(iface_index < UAVCAN_STM32_NUM_IFACES);
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (ifaces[iface_index] != UAVCAN_NULLPTR)
    {
        ifaces[iface_index]->handleTxInterrupt(utc_usec);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

inline void handleRxInterrupt(uavcan::uint8_t iface_index, uavcan::uint8_t fifo_index)
{
    UAVCAN_ASSERT(iface_index < UAVCAN_STM32_NUM_IFACES);
    uavcan::uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();
    if (utc_usec > 0)
    {
        utc_usec--;
    }
    if (ifaces[iface_index] != UAVCAN_NULLPTR)
    {
        ifaces[iface_index]->handleRxInterrupt(fifo_index, utc_usec);
    }
    else
    {
        UAVCAN_ASSERT(0);
    }
}

} // namespace

/*
 * CanIface::RxQueue
 */
void CanIface::RxQueue::registerOverflow()
{
    if (overflow_cnt_ < 0xFFFFFFFF)
    {
        overflow_cnt_++;
    }
}

void CanIface::RxQueue::push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags)
{
    buf_[in_].frame    = frame;
    buf_[in_].utc_usec = utc_usec;
    buf_[in_].flags    = flags;
    in_++;
    if (in_ >= capacity_)
    {
        in_ = 0;
    }
    len_++;
    if (len_ > capacity_)
    {
        len_ = capacity_;
        registerOverflow();
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
    }
}

void CanIface::RxQueue::pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags)
{
    if (len_ > 0)
    {
        out_frame    = buf_[out_].frame;
        out_utc_usec = buf_[out_].utc_usec;
        out_flags    = buf_[out_].flags;
        out_++;
        if (out_ >= capacity_)
        {
            out_ = 0;
        }
        len_--;
    }
    else { UAVCAN_ASSERT(0); }
}

void CanIface::RxQueue::reset()
{
    in_ = 0;
    out_ = 0;
    len_ = 0;
    overflow_cnt_ = 0;
}

/*
 * CanIface
 */
const uavcan::uint32_t CanIface::TSR_ABRQx[CanIface::NumTxMailboxes] =
{
    bxcan::TSR_ABRQ0,
    bxcan::TSR_ABRQ1,
    bxcan::TSR_ABRQ2
};

int CanIface::computeTimings(const uavcan::uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1)
    {
        return -ErrInvalidBitRate;
    }

    /*
     * Hardware configuration
     */
    const uavcan::uint32_t pclk = STM32_PCLK1;

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

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
        if (bs1_bs2_sum <= 2)
        {
            return -ErrInvalidBitRate;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uavcan::uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
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
    struct BsPair
    {
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

        bool isValid() const { return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2); }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uavcan::uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation)
    {
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
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid())
    {
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
    if (frame.isErrorFrame() || frame.dlc > 8)
    {
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
    uavcan::uint8_t txmailbox = 0xFF;
    if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0)
    {
        txmailbox = 0;
    }
    else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1)
    {
        txmailbox = 1;
    }
    else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2)
    {
        txmailbox = 2;
    }
    else
    {
        return 0;       // No transmission for you.
    }

    peak_tx_mailbox_index_ = uavcan::max(peak_tx_mailbox_index_, txmailbox);    // Statistics

    /*
     * Setting up the mailbox
     */
    bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
    if (frame.isExtended())
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
    }
    else
    {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest())
    {
        mb.TIR |= bxcan::TIR_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = (uavcan::uint32_t(frame.data[7]) << 24) |
              (uavcan::uint32_t(frame.data[6]) << 16) |
              (uavcan::uint32_t(frame.data[5]) << 8)  |
              (uavcan::uint32_t(frame.data[4]) << 0);
    mb.TDLR = (uavcan::uint32_t(frame.data[3]) << 24) |
              (uavcan::uint32_t(frame.data[2]) << 16) |
              (uavcan::uint32_t(frame.data[1]) << 8)  |
              (uavcan::uint32_t(frame.data[0]) << 0);

    mb.TIR |= bxcan::TIR_TXRQ;  // Go.

    /*
     * Registering the pending transmission so we can track its deadline and loopback it as needed
     */
    TxItem& txi = pending_tx_[txmailbox];
    txi.deadline       = tx_deadline;
    txi.frame          = frame;
    txi.loopback       = (flags & uavcan::CanIOFlagLoopback) != 0;
    txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    txi.pending        = true;
    return 1;
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                  uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic();  // High precision is not required for monotonic timestamps
    uavcan::uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.getLength() == 0)
        {
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
    if (num_configs <= NumFilters)
    {
        CriticalSectionLocker lock;

        can_->FMR |= bxcan::FMR_FINIT;

        // Slave (CAN2) gets half of the filters
        can_->FMR &= ~0x00003F00UL;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8;

        can_->FFA1R = 0x0AAAAAAA; // FIFO's are interleaved between filters
        can_->FM1R = 0; // Identifier Mask mode
        can_->FS1R = 0x7ffffff; // Single 32-bit for all

        const uint8_t filter_start_index = (self_index_ == 0) ? 0 : NumFilters;

        if (num_configs == 0)
        {
            can_->FilterRegister[filter_start_index].FR1 = 0;
            can_->FilterRegister[filter_start_index].FR2 = 0;
            can_->FA1R = 1 << filter_start_index;
        }
        else
        {
            for (uint8_t i = 0; i < NumFilters; i++)
            {
                if (i < num_configs)
                {
                    uint32_t id   = 0;
                    uint32_t mask = 0;

                    const uavcan::CanFilterConfig* const cfg = filter_configs + i;

                    if ((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF))
                    {
                        id   = (cfg->id   & uavcan::CanFrame::MaskExtID) << 3;
                        mask = (cfg->mask & uavcan::CanFrame::MaskExtID) << 3;
                        id |= bxcan::RIR_IDE;
                    }
                    else
                    {
                        id   = (cfg->id   & uavcan::CanFrame::MaskStdID) << 21;  // Regular std frames, nothing fancy.
                        mask = (cfg->mask & uavcan::CanFrame::MaskStdID) << 21;  // Boring.
                    }

                    if (cfg->id & uavcan::CanFrame::FlagRTR)
                    {
                        id |= bxcan::RIR_RTR;
                    }

                    if (cfg->mask & uavcan::CanFrame::FlagEFF)
                    {
                        mask |= bxcan::RIR_IDE;
                    }

                    if (cfg->mask & uavcan::CanFrame::FlagRTR)
                    {
                        mask |= bxcan::RIR_RTR;
                    }

                    can_->FilterRegister[filter_start_index + i].FR1 = id;
                    can_->FilterRegister[filter_start_index + i].FR2 = mask;

                    can_->FA1R |= (1 << (filter_start_index + i));
                }
                else
                {
                    can_->FA1R &= ~(1 << (filter_start_index + i));
                }
            }
        }

        can_->FMR &= ~bxcan::FMR_FINIT;

        return 0;
    }

    return -ErrFilterNumConfigs;
}

bool CanIface::waitMsrINakBitStateChange(bool target_state)
{
    const unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++)
    {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state)
        {
            return true;
        }
#if CH_KERNEL_MAJOR >= 5
    ::chThdSleep(chTimeMS2I(1));
#else
    ::chThdSleep(MS2ST(1));
#endif
    }
    return false;
}

int CanIface::init(const uavcan::uint32_t bitrate, const OperatingMode mode)
{
    /*
     * We need to silence the controller in the first order, otherwise it may interfere with the following operations.
     */
    {
        CriticalSectionLocker lock;

        can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
        can_->MCR |= bxcan::MCR_INRQ;   // Request init

        can_->IER = 0;                  // Disable interrupts while initialization is in progress
    }

    if (!waitMsrINakBitStateChange(true))
    {
        UAVCAN_STM32_LOG("MSR INAK not set");
        can_->MCR = bxcan::MCR_RESET;
        return -ErrMsrInakNotSet;
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
    if (timings_res < 0)
    {
        can_->MCR = bxcan::MCR_RESET;
        return timings_res;
    }
    UAVCAN_STM32_LOG("Timings: presc=%u sjw=%u bs1=%u bs2=%u",
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

    if (!waitMsrINakBitStateChange(false))
    {
        UAVCAN_STM32_LOG("MSR INAK not cleared");
        can_->MCR = bxcan::MCR_RESET;
        return -ErrMsrInakNotCleared;
    }

    /*
     * Default filter configuration
     */
    if (self_index_ == 0)
    {
        can_->FMR |= bxcan::FMR_FINIT;

        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uavcan::uint32_t>(NumFilters) << 8;  // Slave (CAN2) gets half of the filters

        can_->FFA1R = 0;                           // All assigned to FIFO0 by default
        can_->FM1R = 0;                            // Indentifier Mask mode

#if UAVCAN_STM32_NUM_IFACES > 1
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

    return 0;
}

void CanIface::handleTxMailboxInterrupt(uavcan::uint8_t mailbox_index, bool txok, const uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(mailbox_index < NumTxMailboxes);

    had_activity_ = had_activity_ || txok;

    TxItem& txi = pending_tx_[mailbox_index];

    if (txi.loopback && txok && txi.pending)
    {
        rx_queue_.push(txi.frame, utc_usec, uavcan::CanIOFlagLoopback);
    }

    txi.pending = false;
}

void CanIface::handleTxInterrupt(const uavcan::uint64_t utc_usec)
{
    // TXOK == false means that there was a hardware failure
    if (can_->TSR & bxcan::TSR_RQCP0)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK0;
        can_->TSR = bxcan::TSR_RQCP0;
        handleTxMailboxInterrupt(0, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP1)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK1;
        can_->TSR = bxcan::TSR_RQCP1;
        handleTxMailboxInterrupt(1, txok, utc_usec);
    }
    if (can_->TSR & bxcan::TSR_RQCP2)
    {
        const bool txok = can_->TSR & bxcan::TSR_TXOK2;
        can_->TSR = bxcan::TSR_RQCP2;
        handleTxMailboxInterrupt(2, txok, utc_usec);
    }
    update_event_.signalFromInterrupt();

    pollErrorFlagsFromISR();

    #if UAVCAN_STM32_FREERTOS
    update_event_.yieldFromISR();
    #endif
}

void CanIface::handleRxInterrupt(uavcan::uint8_t fifo_index, uavcan::uint64_t utc_usec)
{
    UAVCAN_ASSERT(fifo_index < 2);

    volatile uavcan::uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0)
    {
        UAVCAN_ASSERT(0);  // Weird, IRQ is here but no data to read
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0)
    {
        error_cnt_++;
    }

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0)
    {
        frame.id = uavcan::CanFrame::MaskStdID & (rf.RIR >> 21);
    }
    else
    {
        frame.id = uavcan::CanFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0)
    {
        frame.id |= uavcan::CanFrame::FlagRTR;
    }

    frame.dlc = rf.RDTR & 15;

    frame.data[0] = uavcan::uint8_t(0xFF & (rf.RDLR >> 0));
    frame.data[1] = uavcan::uint8_t(0xFF & (rf.RDLR >> 8));
    frame.data[2] = uavcan::uint8_t(0xFF & (rf.RDLR >> 16));
    frame.data[3] = uavcan::uint8_t(0xFF & (rf.RDLR >> 24));
    frame.data[4] = uavcan::uint8_t(0xFF & (rf.RDHR >> 0));
    frame.data[5] = uavcan::uint8_t(0xFF & (rf.RDHR >> 8));
    frame.data[6] = uavcan::uint8_t(0xFF & (rf.RDHR >> 16));
    frame.data[7] = uavcan::uint8_t(0xFF & (rf.RDHR >> 24));

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL;  // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    rx_queue_.push(frame, utc_usec, 0);
 #if !HAL_MINIMIZE_FEATURES
    slcan_router().route_frame_to_slcan(this, frame, utc_usec);
#endif
    had_activity_ = true;
    update_event_.signalFromInterrupt();

    pollErrorFlagsFromISR();

    #if UAVCAN_STM32_FREERTOS
    update_event_.yieldFromISR();
    #endif
}

void CanIface::pollErrorFlagsFromISR()
{
    const uavcan::uint8_t lec = uavcan::uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec != 0)
    {
        can_->ESR = 0;
        error_cnt_++;

        // Serving abort requests
        for (int i = 0; i < NumTxMailboxes; i++)    // Dear compiler, may I suggest you to unroll this loop please.
        {
            TxItem& txi = pending_tx_[i];
            if (txi.pending && txi.abort_on_error)
            {
                can_->TSR = TSR_ABRQx[i];
                txi.pending = false;
                served_aborts_cnt_++;
            }
        }
    }
}

void CanIface::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++)
    {
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.deadline < current_time)
        {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            txi.pending = false;
            error_cnt_++;
        }
    }
}

bool CanIface::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
{
    /*
     * We can accept more frames only if the following conditions are satisfied:
     *  - There is at least one TX mailbox free (obvious enough);
     *  - The priority of the new frame is higher than priority of all TX mailboxes.
     */
    {
        static const uavcan::uint32_t TME = bxcan::TSR_TME0 | bxcan::TSR_TME1 | bxcan::TSR_TME2;
        const uavcan::uint32_t tme = can_->TSR & TME;

        if (tme == TME)     // All TX mailboxes are free (as in freedom).
        {
            return true;
        }

        if (tme == 0)       // All TX mailboxes are busy transmitting.
        {
            return false;
        }
    }

    /*
     * The second condition requires a critical section.
     */
    CriticalSectionLocker lock;

    for (int mbx = 0; mbx < NumTxMailboxes; mbx++)
    {
        if (pending_tx_[mbx].pending && !frame.priorityHigherThan(pending_tx_[mbx].frame))
        {
            return false;       // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true;                // This new frame will be added to a free TX mailbox in the next @ref send().
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

        if (pending_tx[i] != UAVCAN_NULLPTR)
        {
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
        (inout_masks.write & in_masks.write) != 0)
    {
        return 1;
    }

    (void)update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx);  // Return what we got even if none of the requested events are set
    return 1;                                   // Return value doesn't matter as long as it is non-negative
}


#if UAVCAN_STM32_BAREMETAL || UAVCAN_STM32_FREERTOS

static void nvicEnableVector(IRQn_Type irq,  uint8_t prio)
{
    #if !defined (USE_HAL_DRIVER)
      NVIC_InitTypeDef NVIC_InitStructure;
      NVIC_InitStructure.NVIC_IRQChannel = irq;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prio;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    #else
      HAL_NVIC_SetPriority(irq, prio, 0);
      HAL_NVIC_EnableIRQ(irq);
    #endif
}

#endif

void CanDriver::initOnce()
{
    /*
     * CAN1, CAN2
     */
    {
        CriticalSectionLocker lock;
        RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
# if UAVCAN_STM32_NUM_IFACES > 1
        RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
        RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
# endif
    }

    /*
     * IRQ
     */
    {
        CriticalSectionLocker lock;
        nvicEnableVector(CAN1_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN1_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# if UAVCAN_STM32_NUM_IFACES > 1
        nvicEnableVector(CAN2_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN2_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        nvicEnableVector(CAN2_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
# endif
    }
}

int CanDriver::init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode)
{
    int res = 0;

    UAVCAN_STM32_LOG("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));

    static bool initialized_once = false;
    if (!initialized_once)
    {
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
    if (res < 0)                                // a typical race condition.
    {
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
    if (res < 0)
    {
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

void CanDriver::initOnce(uavcan::uint8_t can_number, bool enable_irqs)
{
    /*
     * CAN1, CAN2
     */
    {
        CriticalSectionLocker lock;
        if (can_number == 0) {
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN1EN;
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN1RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
        }
# if UAVCAN_STM32_NUM_IFACES > 1
        else if (can_number == 1) {
            RCC->APB1ENR  |=  RCC_APB1ENR_CAN2EN;
            RCC->APB1RSTR |=  RCC_APB1RSTR_CAN2RST;
            RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
        }
# endif
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
            nvicEnableVector(CAN1_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(CAN1_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(CAN1_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
        }
# if UAVCAN_STM32_NUM_IFACES > 1
        else if (can_number == 1) {
            nvicEnableVector(CAN2_TX_IRQn,  UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(CAN2_RX0_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
            nvicEnableVector(CAN2_RX1_IRQn, UAVCAN_STM32_IRQ_PRIORITY_MASK);
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
        if (res < 0)                                // a typical race condition.
        {
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
        if (res < 0)
        {
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
    if (iface_index < num_ifaces_)
    {
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

UAVCAN_STM32_IRQ_HANDLER(CAN1_TX_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_TX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleTxInterrupt(0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX0_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_RX0_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleRxInterrupt(0, 0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN1_RX1_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN1_RX1_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleRxInterrupt(0, 1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

# if UAVCAN_STM32_NUM_IFACES > 1

#if !defined(CAN2_TX_IRQHandler)
# error "Misconfigured build1"
#endif

#if !defined(CAN2_RX0_IRQHandler)
# error "Misconfigured build2"
#endif

#if !defined(CAN2_RX1_IRQHandler)
# error "Misconfigured build3"
#endif

UAVCAN_STM32_IRQ_HANDLER(CAN2_TX_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN2_TX_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleTxInterrupt(1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN2_RX0_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN2_RX0_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleRxInterrupt(1, 0);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

UAVCAN_STM32_IRQ_HANDLER(CAN2_RX1_IRQHandler);
UAVCAN_STM32_IRQ_HANDLER(CAN2_RX1_IRQHandler)
{
    UAVCAN_STM32_IRQ_PROLOGUE();
    ChibiOS_CAN::handleRxInterrupt(1, 1);
    UAVCAN_STM32_IRQ_EPILOGUE();
}

# endif

} // extern "C"

#endif //HAL_WITH_UAVCAN
