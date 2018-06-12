/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * With modifications for Ardupilot CAN driver
 * Copyright (C) 2017 Eugene Shamaev
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#if HAL_WITH_UAVCAN

#include <cassert>
#include <cstring>
#include "CAN.h"

#include <nuttx/irq.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <arch/board/board.h>

#include "Scheduler.h"

/*
 * FOR INVESTIGATION:
 * AP_HAL::micros64() was called for monotonic time counter
 * pavel-kirienko: This will work as long as we don't need to synchronize the autopilot's own clock with an external
 * time base, e.g. a GNSS time provided by an external GNSS receiver. Libuavcan's STM32 driver supports automatic time
 * synchronization only if it has a dedicated hardware timer to work with.
 */

extern const AP_HAL::HAL& hal;

#include <AP_UAVCAN/AP_UAVCAN.h>

extern "C" {
    static int can1_irq(const int irq, void*);
#if CAN_STM32_NUM_IFACES > 1
    static int can2_irq(const int irq, void*);
#endif
}

using namespace VRBRAIN;

uint64_t clock::getUtcUSecFromCanInterrupt()
{
    return AP_HAL::micros64();
}

uavcan::MonotonicTime clock::getMonotonic()
{
    return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
}

BusEvent::BusEvent(VRBRAINCANManager& can_driver) :
    _signal(0)
{
    sem_init(&_wait_semaphore, 0, 0);
}

BusEvent::~BusEvent()
{
}

bool BusEvent::wait(uavcan::MonotonicDuration duration)
{
    struct hrt_call wait_call;

    irqstate_t irs = irqsave();
    if (_signal) {
        _signal = 0;
        irqrestore(irs);
        return true;
    }

    sem_init(&_wait_semaphore, 0, 0);
    irqrestore(irs);

    hrt_call_after(&wait_call, duration.toUSec(), (hrt_callout) signalFromCallOut, this);
    sem_wait(&_wait_semaphore);

    hrt_cancel(&wait_call);

    irs = irqsave();
    if (_signal) {
        _signal = 0;
        irqrestore(irs);

        return true;
    }
    irqrestore(irs);

    return false;
}

void BusEvent::signalFromCallOut(BusEvent *sem)
{
    sem_post(&sem->_wait_semaphore);
}

void BusEvent::signalFromInterrupt()
{
    _signal++;
    sem_post(&_wait_semaphore);
}

static void handleTxInterrupt(uint8_t iface_index)
{
    if (iface_index >= CAN_STM32_NUM_IFACES) {
        return;
    }
    uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (hal.can_mgr[i] == nullptr) {
            continue;
        }
        VRBRAINCAN* iface = ((VRBRAINCANManager*) hal.can_mgr[i])->getIface_out_to_in(iface_index);
        if (iface != nullptr) {
            iface->handleTxInterrupt(utc_usec);
        }
    }
}

static void handleRxInterrupt(uint8_t iface_index, uint8_t fifo_index)
{
    if (iface_index >= CAN_STM32_NUM_IFACES) {
        return;
    }
    uint64_t utc_usec = clock::getUtcUSecFromCanInterrupt();

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        if (hal.can_mgr[i] == nullptr) {
            continue;
        }
        VRBRAINCAN* iface = ((VRBRAINCANManager*) hal.can_mgr[i])->getIface_out_to_in(iface_index);
        if (iface != nullptr) {
            iface->handleRxInterrupt(fifo_index, utc_usec);
        }
    }
}

const uint32_t VRBRAINCAN::TSR_ABRQx[VRBRAINCAN::NumTxMailboxes] = { bxcan::TSR_ABRQ0, bxcan::TSR_ABRQ1, bxcan::TSR_ABRQ2 };

int VRBRAINCAN::computeTimings(const uint32_t target_bitrate, Timings& out_timings)
{
    if (target_bitrate < 1) {
        return -ErrInvalidBitRate;
    }

    /*
     * Hardware configuration
     */
    const uint32_t pclk = STM32_PCLK1_FREQUENCY;

    static const uint8_t MaxBS1 = 16;
    static const uint8_t MaxBS2 = 8;

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
    const uint8_t max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    if (max_quanta_per_bit > (MaxBS1 + MaxBS2)) {
        if (AP_BoardConfig_CAN::get_can_debug() >= 1) {
            printf("VRBRAINCAN::computeTimings max_quanta_per_bit problem\n\r");
        }
    }

    static const uint16_t MaxSamplePointLocation = 900;

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
            return -ErrInvalidBitRate;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
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
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair() :
            bs1(0), bs2(0), sample_point_permill(0)
        {
        }

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) :
            bs1(arg_bs1), bs2(uint8_t(bs1_bs2_sum - bs1)), sample_point_permill(
                uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {
            if (bs1_bs2_sum <= arg_bs1) {
                if (AP_BoardConfig_CAN::get_can_debug() >= 1) {
                    AP_HAL::panic("VRBRAINCAN::computeTimings bs1_bs2_sum <= arg_bs1");
                }
            }
        }

        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation || !solution.isValid()) {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));

        if (!solution.isValid())
        {
            printf("VRBRAINCAN::computeTimings second solution invalid\n\r");
            return -ErrLogic;
        }
    }

    /*
     * Final validation
     */
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid()) {
        if (AP_BoardConfig_CAN::get_can_debug() >= 1) {
            printf("VRBRAINCAN::computeTimings target_bitrate error\n\r");
        }
        return -ErrLogic;
    }

    if (AP_BoardConfig_CAN::get_can_debug() >= 2) {
        printf("VRBRAINCAN::computeTimings Timings: quanta/bit: %d, sample point location: %.1f%%\n\r",
               int(1 + solution.bs1 + solution.bs2), double(solution.sample_point_permill / 10.0));
    }

    out_timings.prescaler = uint16_t(prescaler - 1U);
    out_timings.sjw = 0;                                      // Which means one
    out_timings.bs1 = uint8_t(solution.bs1 - 1);
    out_timings.bs2 = uint8_t(solution.bs2 - 1);

    return 0;
}

int16_t VRBRAINCAN::send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags)
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
    uint8_t txmailbox = 0xFF;
    if ((can_->TSR & bxcan::TSR_TME0) == bxcan::TSR_TME0) {
        txmailbox = 0;
    } else if ((can_->TSR & bxcan::TSR_TME1) == bxcan::TSR_TME1) {
        txmailbox = 1;
    } else if ((can_->TSR & bxcan::TSR_TME2) == bxcan::TSR_TME2) {
        txmailbox = 2;
    } else {
        return 0;       // No transmission for you.
    }

    peak_tx_mailbox_index_ = uavcan::max(peak_tx_mailbox_index_, txmailbox); // Statistics

    /*
     * Setting up the mailbox
     */
    bxcan::TxMailboxType& mb = can_->TxMailbox[txmailbox];
    if (frame.isExtended()) {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskExtID) << 3) | bxcan::TIR_IDE;
    } else {
        mb.TIR = ((frame.id & uavcan::CanFrame::MaskStdID) << 21);
    }

    if (frame.isRemoteTransmissionRequest()) {
        mb.TIR |= bxcan::TIR_RTR;
    }

    mb.TDTR = frame.dlc;

    mb.TDHR = (uint32_t(frame.data[7]) << 24) | (uint32_t(frame.data[6]) << 16) | (uint32_t(frame.data[5]) << 8)
              | (uint32_t(frame.data[4]) << 0);
    mb.TDLR = (uint32_t(frame.data[3]) << 24) | (uint32_t(frame.data[2]) << 16) | (uint32_t(frame.data[1]) << 8)
              | (uint32_t(frame.data[0]) << 0);

    mb.TIR |= bxcan::TIR_TXRQ;  // Go.

    /*
     * Registering the pending transmission so we can track its deadline and loopback it as needed
     */
    TxItem& txi = pending_tx_[txmailbox];
    txi.deadline = tx_deadline;
    txi.frame = frame;
    txi.loopback = (flags & uavcan::CanIOFlagLoopback) != 0;
    txi.abort_on_error = (flags & uavcan::CanIOFlagAbortOnError) != 0;
    txi.pending = true;
    return 1;
}

int16_t VRBRAINCAN::receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                        uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)
{
    out_ts_monotonic = clock::getMonotonic(); // High precision is not required for monotonic timestamps
    uint64_t utc_usec = 0;
    {
        CriticalSectionLocker lock;
        if (rx_queue_.available() == 0) {
            return 0;
        }

        CanRxItem frm;
        rx_queue_.pop(frm);
        out_frame = frm.frame;
        utc_usec = frm.utc_usec;
        out_flags = frm.flags;
    }
    out_ts_utc = uavcan::UtcTime::fromUSec(utc_usec);
    return 1;
}

int16_t VRBRAINCAN::configureFilters(const uavcan::CanFilterConfig* filter_configs, uint16_t num_configs)
{
    if (num_configs > NumFilters) {
        return -ErrFilterNumConfigs;
    }

    CriticalSectionLocker lock;

    can_->FMR |= bxcan::FMR_FINIT;

    // Slave (CAN2) gets half of the filters
    can_->FMR = (can_->FMR & ~0x00003F00) | static_cast<uint32_t>(NumFilters) << 8;

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

                const uavcan::CanFilterConfig* const cfg = filter_configs + i;

                if ((cfg->id & uavcan::CanFrame::FlagEFF) || !(cfg->mask & uavcan::CanFrame::FlagEFF)) {
                    id   = (cfg->id   & uavcan::CanFrame::MaskExtID) << 3;
                    mask = (cfg->mask & uavcan::CanFrame::MaskExtID) << 3;
                    id |= bxcan::RIR_IDE;
                } else {
                    id   = (cfg->id   & uavcan::CanFrame::MaskStdID) << 21;
                    mask = (cfg->mask & uavcan::CanFrame::MaskStdID) << 21;
                }

                if (cfg->id & uavcan::CanFrame::FlagRTR) {
                    id |= bxcan::RIR_RTR;
                }

                if (cfg->mask & uavcan::CanFrame::FlagEFF) {
                    mask |= bxcan::RIR_IDE;
                }

                if (cfg->mask & uavcan::CanFrame::FlagRTR) {
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

    return 0;
}

bool VRBRAINCAN::waitMsrINakBitStateChange(bool target_state)
{
    const unsigned Timeout = 1000;
    for (unsigned wait_ack = 0; wait_ack < Timeout; wait_ack++) {
        const bool state = (can_->MSR & bxcan::MSR_INAK) != 0;
        if (state == target_state) {
            return true;
        }
        hal.scheduler->delay_microseconds(1000);
    }
    return false;
}

int VRBRAINCAN::init(const uint32_t bitrate, const OperatingMode mode)
{
    /* We need to silence the controller in the first order, otherwise it may interfere with the following operations. */

    {
        CriticalSectionLocker lock;

        can_->MCR &= ~bxcan::MCR_SLEEP; // Exit sleep mode
        can_->MCR |= bxcan::MCR_INRQ; // Request init

        can_->IER = 0; // Disable CAN interrupts while initialization is in progress
    }

    if (!waitMsrINakBitStateChange(true)) {
        if (AP_BoardConfig_CAN::get_can_debug() >= 1) {
            printf("VRBRAINCAN::init MSR INAK not set\n\r");
        }
        can_->MCR = bxcan::MCR_RESET;
        return -ErrMsrInakNotSet;
    }

    /*
     * Object state - CAN interrupts are disabled, so it's safe to modify it now
     */
    rx_queue_.clear();
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
        can_->MCR = bxcan::MCR_RESET;
        return timings_res;
    }
    if (AP_BoardConfig_CAN::get_can_debug() >= 2) {
        printf("VRBRAINCAN::init Timings: presc=%u sjw=%u bs1=%u bs2=%u\n\r", unsigned(timings.prescaler),
               unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));
    }

    /*
     * Hardware initialization (the hardware has already confirmed initialization mode, see above)
     */
    can_->MCR = bxcan::MCR_ABOM | bxcan::MCR_AWUM | bxcan::MCR_INRQ; // RM page 648

    can_->BTR = ((timings.sjw & 3U) << 24) | ((timings.bs1 & 15U) << 16) | ((timings.bs2 & 7U) << 20)
                | (timings.prescaler & 1023U) | ((mode == SilentMode) ? bxcan::BTR_SILM : 0);

    can_->IER = bxcan::IER_TMEIE | // TX mailbox empty
                bxcan::IER_FMPIE0 | // RX FIFO 0 is not empty
                bxcan::IER_FMPIE1; // RX FIFO 1 is not empty

    can_->MCR &= ~bxcan::MCR_INRQ; // Leave init mode

    if (!waitMsrINakBitStateChange(false)) {
        if (AP_BoardConfig_CAN::get_can_debug() >= 1) {
            printf("VRBRAINCAN::init MSR INAK not cleared\n\r");
        }
        can_->MCR = bxcan::MCR_RESET;
        return -ErrMsrInakNotCleared;
    }

    /*
     * Default filter configuration
     */
    if (self_index_ == 0) {
        can_->FMR |= bxcan::FMR_FINIT;

        can_->FMR &= 0xFFFFC0F1;
        can_->FMR |= static_cast<uint32_t>(NumFilters) << 8; // Slave (CAN2) gets half of the filters

        can_->FFA1R = 0; // All assigned to FIFO0 by default
        can_->FM1R = 0; // Indentifier Mask mode

#if CAN_STM32_NUM_IFACES > 1
        can_->FS1R = 0x7ffffff; // Single 32-bit for all
        can_->FilterRegister[0].FR1 = 0; // CAN1 accepts everything
        can_->FilterRegister[0].FR2 = 0;
        can_->FilterRegister[NumFilters].FR1 = 0; // CAN2 accepts everything
        can_->FilterRegister[NumFilters].FR2 = 0;
        can_->FA1R = 1 | (1 << NumFilters); // One filter per each iface
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

void VRBRAINCAN::handleTxMailboxInterrupt(uint8_t mailbox_index, bool txok, const uint64_t utc_usec)
{
    if (mailbox_index >= NumTxMailboxes) {
        return;
    }

    had_activity_ = had_activity_ || txok;

    TxItem& txi = pending_tx_[mailbox_index];

    if (txi.loopback && txok && txi.pending) {
        CanRxItem frm;
        frm.frame = txi.frame;
        frm.flags = uavcan::CanIOFlagLoopback;
        frm.utc_usec = utc_usec;
        rx_queue_.push(frm);
    }

    txi.pending = false;
}

void VRBRAINCAN::handleTxInterrupt(const uint64_t utc_usec)
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

    if(update_event_ != nullptr) {
        update_event_->signalFromInterrupt();
    }

    pollErrorFlagsFromISR();
}

void VRBRAINCAN::handleRxInterrupt(uint8_t fifo_index, uint64_t utc_usec)
{
    if (fifo_index >= 2) {
        return;
    }

    volatile uint32_t* const rfr_reg = (fifo_index == 0) ? &can_->RF0R : &can_->RF1R;
    if ((*rfr_reg & bxcan::RFR_FMP_MASK) == 0) {
        return;
    }

    /*
     * Register overflow as a hardware error
     */
    if ((*rfr_reg & bxcan::RFR_FOVR) != 0) {
        error_cnt_++;
    }

    /*
     * Read the frame contents
     */
    uavcan::CanFrame frame;
    const bxcan::RxMailboxType& rf = can_->RxMailbox[fifo_index];

    if ((rf.RIR & bxcan::RIR_IDE) == 0) {
        frame.id = uavcan::CanFrame::MaskStdID & (rf.RIR >> 21);
    } else {
        frame.id = uavcan::CanFrame::MaskExtID & (rf.RIR >> 3);
        frame.id |= uavcan::CanFrame::FlagEFF;
    }

    if ((rf.RIR & bxcan::RIR_RTR) != 0) {
        frame.id |= uavcan::CanFrame::FlagRTR;
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

    *rfr_reg = bxcan::RFR_RFOM | bxcan::RFR_FOVR | bxcan::RFR_FULL; // Release FIFO entry we just read

    /*
     * Store with timeout into the FIFO buffer and signal update event
     */
    CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.utc_usec = utc_usec;
    rx_queue_.push(frm);

    had_activity_ = true;
    if(update_event_ != nullptr) {
        update_event_->signalFromInterrupt();
    }

    pollErrorFlagsFromISR();
}

void VRBRAINCAN::pollErrorFlagsFromISR()
{
    const uint8_t lec = uint8_t((can_->ESR & bxcan::ESR_LEC_MASK) >> bxcan::ESR_LEC_SHIFT);
    if (lec == 0) {
        return;
    }
    can_->ESR = 0;
    error_cnt_++;

    // Serving abort requests
    for (int i = 0; i < NumTxMailboxes; i++) { // Dear compiler, may I suggest you to unroll this loop please.
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.abort_on_error) {
            can_->TSR = TSR_ABRQx[i];
            txi.pending = false;
            served_aborts_cnt_++;
        }
    }
}

void VRBRAINCAN::discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time)
{
    CriticalSectionLocker lock;
    for (int i = 0; i < NumTxMailboxes; i++) {
        TxItem& txi = pending_tx_[i];
        if (txi.pending && txi.deadline < current_time) {
            can_->TSR = TSR_ABRQx[i];  // Goodnight sweet transmission
            txi.pending = false;
            error_cnt_++;
        }
    }
}

bool VRBRAINCAN::canAcceptNewTxFrame(const uavcan::CanFrame& frame) const
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
        if (pending_tx_[mbx].pending && !frame.priorityHigherThan(pending_tx_[mbx].frame)) {
            return false; // There's a mailbox whose priority is higher or equal the priority of the new frame.
        }
    }

    return true; // This new frame will be added to a free TX mailbox in the next @ref send().
}

bool VRBRAINCAN::isRxBufferEmpty() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available() == 0;
}

uint64_t VRBRAINCAN::getErrorCount() const
{
    CriticalSectionLocker lock;
    return error_cnt_;
    //TODO: + rx_queue_.getOverflowCount();
}

unsigned VRBRAINCAN::getRxQueueLength() const
{
    CriticalSectionLocker lock;
    return rx_queue_.available();
}

bool VRBRAINCAN::hadActivity()
{
    CriticalSectionLocker lock;
    const bool ret = had_activity_;
    had_activity_ = false;
    return ret;
}

bool VRBRAINCAN::begin(uint32_t bitrate)
{
    if (init(bitrate, OperatingMode::NormalMode) == 0) {
        bitrate_ = bitrate;
        initialized_ = true;
    } else {
        initialized_ = false;
    }
    return initialized_;
}

void VRBRAINCAN::reset()
{
    if (initialized_ && bitrate_ != 0) {
        init(bitrate_, OperatingMode::NormalMode);
    }
}

bool VRBRAINCAN::is_initialized()
{
    return initialized_;
}

int32_t VRBRAINCAN::available()
{
    if (initialized_) {
        return getRxQueueLength();
    } else {
        return -1;
    }
}

int32_t VRBRAINCAN::tx_pending()
{
    if (!initialized_) {
        return -1;
    }

    int32_t ret = 0;
    {
        CriticalSectionLocker lock;

        for (int mbx = 0; mbx < NumTxMailboxes; mbx++) {
            if (pending_tx_[mbx].pending) {
                ret++;
            }
        }
    }
    return ret;
}

/*
 * CanDriver
 */

VRBRAINCANManager::VRBRAINCANManager() :
    update_event_(*this), if0_(bxcan::Can[0], nullptr, 0, CAN_STM32_RX_QUEUE_SIZE), if1_(
    bxcan::Can[1], nullptr, 1, CAN_STM32_RX_QUEUE_SIZE), initialized_(false), p_uavcan(nullptr)
{
    uavcan::StaticAssert<(CAN_STM32_RX_QUEUE_SIZE <= VRBRAINCAN::MaxRxQueueCapacity)>::check();

    for(uint8_t i = 0; i < CAN_STM32_NUM_IFACES; i++) {
        _ifaces_out_to_in[i] = UINT8_MAX;
    }
}

uavcan::CanSelectMasks VRBRAINCANManager::makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces]) const
{
    uavcan::CanSelectMasks msk;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] == nullptr) {
            continue;
        }
        if (!ifaces[i]->isRxBufferEmpty()) {
            msk.read |= 1 << i;
        }

        if (pending_tx[i] == nullptr) {
            continue;
        }

        if (ifaces[i]->canAcceptNewTxFrame(*pending_tx[i])) {
            msk.write |= 1 << i;
        }
    }

    return msk;
}

bool VRBRAINCANManager::hasReadableInterfaces() const
{
    bool ret = false;

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] != nullptr) {
            ret |= !ifaces[i]->isRxBufferEmpty();
        }
    }

    return ret;
}

int16_t VRBRAINCANManager::select(uavcan::CanSelectMasks& inout_masks,
                              const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces], const uavcan::MonotonicTime blocking_deadline)
{
    const uavcan::CanSelectMasks in_masks = inout_masks;
    const uavcan::MonotonicTime time = clock::getMonotonic();

    for (uint8_t i = 0; i < _ifaces_num; i++) {
        if (ifaces[i] == nullptr) {
            continue;
        }
        ifaces[i]->discardTimedOutTxMailboxes(time);
        {
            CriticalSectionLocker cs_locker;
            ifaces[i]->pollErrorFlagsFromISR();
        }
    }

    inout_masks = makeSelectMasks(pending_tx); // Check if we already have some of the requested events
    if ((inout_masks.read & in_masks.read) != 0 || (inout_masks.write & in_masks.write) != 0) {
        return 1;
    }

    (void) update_event_.wait(blocking_deadline - time); // Block until timeout expires or any iface updates
    inout_masks = makeSelectMasks(pending_tx); // Return what we got even if none of the requested events are set
    return 1; // Return value doesn't matter as long as it is non-negative
}

void VRBRAINCANManager::initOnce(uint8_t can_number)
{
    {
        CriticalSectionLocker lock;
        if (can_number == 0) {
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_CAN1EN);
        }
#if CAN_STM32_NUM_IFACES > 1
        if (can_number == 1) {
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_CAN2EN);
        }
#endif
    }

    if (can_number == 0) {
#if defined(GPIO_CAN1_RX) && defined(GPIO_CAN1_TX)
        stm32_configgpio(GPIO_CAN1_RX);
        stm32_configgpio(GPIO_CAN1_TX);
#else
# error  "Need to define GPIO_CAN1_RX/TX"
#endif
    }
#if CAN_STM32_NUM_IFACES > 1
    if (can_number == 1) {
#if defined(GPIO_CAN2_RX) && defined(GPIO_CAN2_TX)
        stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
        stm32_configgpio(GPIO_CAN2_TX);
#else
# error  "Need to define GPIO_CAN2_RX/TX"
#endif // defined(GPIO_CAN2_RX) && defined(GPIO_CAN2_TX)
    }
#endif // CAN_STM32_NUM_IFACES > 1

    /*
     * IRQ
     */
    if (can_number == 0) {
#if defined(STM32_IRQ_CAN1TX) && defined(STM32_IRQ_CAN1RX0) && defined(STM32_IRQ_CAN1RX1)
        CAN_IRQ_ATTACH(STM32_IRQ_CAN1TX, can1_irq);
        CAN_IRQ_ATTACH(STM32_IRQ_CAN1RX0, can1_irq);
        CAN_IRQ_ATTACH(STM32_IRQ_CAN1RX1, can1_irq);
#else
# error  "Need to define STM32_IRQ_CAN1TX/RX0/RX1"
#endif
    }

#if CAN_STM32_NUM_IFACES > 1
    if (can_number == 1) {
#if defined(STM32_IRQ_CAN2TX) && defined(STM32_IRQ_CAN2RX0) && defined(STM32_IRQ_CAN2RX1)
        CAN_IRQ_ATTACH(STM32_IRQ_CAN2TX, can2_irq);
        CAN_IRQ_ATTACH(STM32_IRQ_CAN2RX0, can2_irq);
        CAN_IRQ_ATTACH(STM32_IRQ_CAN2RX1, can2_irq);
#else
# error  "Need to define STM32_IRQ_CAN2TX/RX0/RX1"
#endif // defined(STM32_IRQ_CAN2TX) && defined(STM32_IRQ_CAN2RX0) && defined(STM32_IRQ_CAN2RX1)
    }
#endif // CAN_STM32_NUM_IFACES > 1
}

int VRBRAINCANManager::init(const uint32_t bitrate, const VRBRAINCAN::OperatingMode mode, uint8_t can_number)
{
    static bool initialized_once[CAN_STM32_NUM_IFACES];

    if (can_number >= CAN_STM32_NUM_IFACES) {
        return -ErrNotImplemented;
    }

    int res = 0;

    if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
        printf("VRBRAINCANManager::init Bitrate %lu mode %d bus %d\n\r", static_cast<unsigned long>(bitrate),
               static_cast<int>(mode), static_cast<int>(can_number));
    }

    // If this outside physical interface was never inited - do this and add it to in/out conversion tables
    if (!initialized_once[can_number]) {
        initialized_once[can_number] = true;
        _ifaces_num++;
        _ifaces_out_to_in[can_number] = _ifaces_num - 1;

        if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
            printf("VRBRAINCANManager::init First initialization bus %d\n\r", static_cast<int>(can_number));
        }

        initOnce(can_number);
    }

    /*
     * CAN1
     */
    if (can_number == 0) {
        if (AP_BoardConfig_CAN::get_can_debug(0) >= 2) {
            printf("VRBRAINCANManager::init Initing iface 0...\n\r");
        }
        ifaces[_ifaces_out_to_in[can_number]] = &if0_;               // This link must be initialized first,
    }

#if CAN_STM32_NUM_IFACES > 1
    /*
     * CAN2
     */
    if (can_number == 1) {
        if (AP_BoardConfig_CAN::get_can_debug(1) >= 2) {
            printf("VRBRAINCANManager::init Initing iface 1...\n\r");
        }
        ifaces[_ifaces_out_to_in[can_number]] = &if1_;                          // Same thing here.
    }
#endif

    ifaces[_ifaces_out_to_in[can_number]]->set_update_event(&update_event_);
    res = ifaces[_ifaces_out_to_in[can_number]]->init(bitrate, mode);
    if (res < 0) {
        ifaces[_ifaces_out_to_in[can_number]] = nullptr;
        return res;
    }

    if (AP_BoardConfig_CAN::get_can_debug(can_number) >= 2) {
        printf("VRBRAINCANManager::init CAN drv init OK, res = %d\n\r", res);
    }

    return res;
}

VRBRAINCAN* VRBRAINCANManager::getIface(uint8_t iface_index)
{
    if (iface_index < _ifaces_num) {
        return ifaces[iface_index];
    }

    return nullptr;
}

VRBRAINCAN* VRBRAINCANManager::getIface_out_to_in(uint8_t iface_index)
{
    // Find which internal interface corresponds to required outside physical interface
    if (iface_index < CAN_STM32_NUM_IFACES) {
        if (_ifaces_out_to_in[iface_index] != UINT8_MAX) {
            return ifaces[_ifaces_out_to_in[iface_index]];
        }
    }

    return nullptr;
}

bool VRBRAINCANManager::hadActivity()
{
    bool ret = false;

    // Go through all interfaces that are present in this manager
    for (uint8_t i = 0; i < _ifaces_num; i++)
    {
        if (ifaces[i] != nullptr) {
            ret |= ifaces[i]->hadActivity();
        }
    }

    return ret;
}

bool VRBRAINCANManager::begin(uint32_t bitrate, uint8_t can_number)
{
    // Try to init outside physical interface 'can_number'
    if (init(bitrate, VRBRAINCAN::OperatingMode::NormalMode, can_number) >= 0) {
        return true;
    }

    return false;
}

bool VRBRAINCANManager::is_initialized()
{
    return initialized_;
}

void VRBRAINCANManager::initialized(bool val)
{
    initialized_ = val;
}

AP_UAVCAN *VRBRAINCANManager::get_UAVCAN(void)
{
    return p_uavcan;
}

void VRBRAINCANManager::set_UAVCAN(AP_UAVCAN *uavcan)
{
    p_uavcan = uavcan;
}

/*
 * Interrupt handlers
 */
extern "C" {
    static int can1_irq(const int irq, void*)
    {
        if (irq == STM32_IRQ_CAN1TX) {
            handleTxInterrupt(0);
        } else if (irq == STM32_IRQ_CAN1RX0) {
            handleRxInterrupt(0, 0);
        } else if (irq == STM32_IRQ_CAN1RX1) {
            handleRxInterrupt(0, 1);
        } else {
            printf("can1_irq unhandled");
        }
        return 0;
    }

#if CAN_STM32_NUM_IFACES > 1
    static int can2_irq(const int irq, void*)
    {
        if (irq == STM32_IRQ_CAN2TX) {
            handleTxInterrupt(1);
        } else if (irq == STM32_IRQ_CAN2RX0) {
            handleRxInterrupt(1, 0);
        } else if (irq == STM32_IRQ_CAN2RX1) {
            handleRxInterrupt(1, 1);
        } else {
            printf("can2_irq unhandled");
        }
        return 0;
    }
#endif

} // extern "C"

#endif
