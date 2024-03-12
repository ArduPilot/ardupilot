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

# if defined(STM32H7XX) || defined(STM32G4)
#include "CANFDIface.h"

#define FDCAN1_IT0_IRQHandler      STM32_FDCAN1_IT0_HANDLER
#define FDCAN1_IT1_IRQHandler      STM32_FDCAN1_IT1_HANDLER
#define FDCAN2_IT0_IRQHandler      STM32_FDCAN2_IT0_HANDLER
#define FDCAN2_IT1_IRQHandler      STM32_FDCAN2_IT1_HANDLER

// FIFO elements are spaced at 18 words
#define FDCAN_FRAME_BUFFER_SIZE 18


//Message RAM Allocations in Word lengths

#if defined(STM32H7)
#define MAX_FILTER_LIST_SIZE 78U            //78 element Standard Filter List elements or 40 element Extended Filter List
#define FDCAN_NUM_RXFIFO0_SIZE 108U         //6 Frames
#define FDCAN_TX_FIFO_BUFFER_SIZE 126U      //7 Frames
#define MESSAGE_RAM_END_ADDR 0x4000B5FC

#elif defined(STM32G4)
#define MAX_FILTER_LIST_SIZE 80U            //80 element Standard Filter List elements or 40 element Extended Filter List
#define FDCAN_NUM_RXFIFO0_SIZE 104U         //26 Frames
#define FDCAN_TX_FIFO_BUFFER_SIZE 128U      //32 Frames
#define FDCAN_MESSAGERAM_STRIDE 0x350       // separation of messageram areas
#define FDCAN_EXFILTER_OFFSET 0x70
#define FDCAN_RXFIFO0_OFFSET 0xB0
#define FDCAN_RXFIFO1_OFFSET 0x188
#define FDCAN_TXFIFO_OFFSET 0x278

#define MESSAGE_RAM_END_ADDR 0x4000B5FC

#else
#error "Unsupported MCU for FDCAN"
#endif

extern const AP_HAL::HAL& hal;

#define STR(x) #x
#define XSTR(x) STR(x)
#if !defined(HAL_LLD_USE_CLOCK_MANAGEMENT)
static_assert(STM32_FDCANCLK == 80U*1000U*1000U, "FDCAN clock must be 80MHz, got " XSTR(STM32_FDCANCLK));
#endif

using namespace ChibiOS;

#if HAL_CANMANAGER_ENABLED
#define Debug(fmt, args...) do { AP::can().log_text(AP_CANManager::LOG_DEBUG, "CANFDIface", fmt, ##args); } while (0)
#else
#define Debug(fmt, args...)
#endif

constexpr CANIface::CanType* const CANIface::Can[];
static ChibiOS::CANIface* can_ifaces[HAL_NUM_CAN_IFACES];

uint8_t CANIface::next_interface;

// mapping from logical interface to physical. First physical is 0, first logical is 0
static constexpr uint8_t can_interfaces[HAL_NUM_CAN_IFACES] = { HAL_CAN_INTERFACE_LIST };

// mapping from physical interface back to logical. First physical is 0, first logical is 0
static constexpr int8_t can_iface_to_idx[3] = { HAL_CAN_INTERFACE_REV_LIST };

#define REG_SET_TIMEOUT 250 // if it takes longer than 250ms for setting a register we have failed

static inline bool driver_initialised(uint8_t iface_index)
{
    if (can_ifaces[iface_index] == nullptr) {
        return false;
    }
    return true;
}

static inline void handleCANInterrupt(uint8_t phys_index, uint8_t line_index)
{
    const int8_t iface_index = can_iface_to_idx[phys_index];
    if (iface_index < 0 || iface_index >= HAL_NUM_CAN_IFACES) {
        return;
    }
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

// constructor suitable for array
CANIface::CANIface() :
    CANIface(next_interface++)
{}

void CANIface::handleBusOffInterrupt()
{
    _detected_bus_off = true;
}

bool CANIface::computeTimings(const uint32_t target_bitrate, Timings& out_timings) const
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
        Debug("Timings: No Solution found\n");
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

    Debug("Timings: quanta/bit: %d, sample point location: %.1f%%\n",
          int(1 + solution.bs1 + solution.bs2), float(solution.sample_point_permill) / 10.F);

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
        Debug("Timings: Invalid Solution %lu %lu %d %d %lu \n", pclk, prescaler, int(solution.bs1), int(solution.bs2), (pclk / (prescaler * (1 + solution.bs1 + solution.bs2))));
        return false;
    }

    out_timings.sample_point_permill = solution.sample_point_permill;
    out_timings.prescaler = uint16_t(prescaler);
    out_timings.sjw = 1;
    out_timings.bs1 = uint8_t(solution.bs1);
    out_timings.bs2 = uint8_t(solution.bs2);
    return true;
}

/*
  table driven timings for CANFD
  These timings are from https://www.kvaser.com/support/calculators/can-fd-bit-timing-calculator
 */
bool CANIface::computeFDTimings(const uint32_t target_bitrate, Timings& out_timings) const
{
    static const struct {
        uint8_t bitrate_mbaud;
        uint8_t prescaler;
        uint8_t bs1;
        uint8_t bs2;
        uint8_t sjw;
        uint8_t sample_point_pct;
    } CANFD_timings[] {
        { 1, 4, 14, 5, 5, 75},
        { 2, 2, 14, 5, 5, 75},
        { 4, 1, 14, 5, 5, 75},
        { 5, 1, 11, 4, 4, 75},
        { 8, 1,  6, 3, 3, 70},
    };
    for (const auto &t : CANFD_timings) {
        if (t.bitrate_mbaud*1000U*1000U == target_bitrate) {
            // out_timings has the register bits, which are the actual value minus 1
            out_timings.prescaler = t.prescaler;
            out_timings.bs1 = t.bs1;
            out_timings.bs2 = t.bs2;
            out_timings.sjw = t.sjw;
            out_timings.sample_point_permill = t.sample_point_pct*10;
            return true;
        }
    }
    return false;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                       CanIOFlags flags)
{
    if (!initialised_) {
        return -1;
    }

    stats.tx_requests++;
    if (frame.isErrorFrame() || (frame.dlc > 8 && !frame.isCanFDFrame()) ||
        frame.dlc > 15) {
        stats.tx_rejected++;
        return -1;
    }

    {
        CriticalSectionLocker lock;

        /*
         * Seeking for an empty slot
         */
        uint8_t index;

        if ((can_->TXFQS & FDCAN_TXFQS_TFQF) != 0) {
            stats.tx_overflow++;
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

        if (frame.isCanFDFrame()) {
            buffer[1] |= FDF | BRS; // do CAN FD transfer and bit rate switching
            stats.fdf_tx_requests++;
            pending_tx_[index].canfd_frame = true;
        } else {
            pending_tx_[index].canfd_frame = false;
        }

        // Write Frame to the message RAM
        const uint8_t data_length = AP_HAL::CANFrame::dlcToDataLength(frame.dlc);
        uint32_t *data_ptr = &buffer[2];
        for (uint8_t i = 0; i < (data_length+3)/4; i++) {
            data_ptr[i] = frame.data_32[i];
        }

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
    }

    // also send on MAVCAN, but don't consider it an error if we can't get the MAVCAN out
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    {
        CriticalSectionLocker lock;
        CanRxItem rx_item;
        if (!rx_queue_.pop(rx_item) || !initialised_) {
            return 0;
        }
        out_frame    = rx_item.frame;
        out_timestamp_us = rx_item.timestamp_us;
        out_flags    = rx_item.flags;
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

bool CANIface::configureFilters(const CanFilterConfig* filter_configs,
                                uint16_t num_configs)
{
    // only enable filters in AP_Periph. It makes no sense on the flight controller
#if !defined(HAL_BUILD_AP_PERIPH) || defined(STM32G4)
    // no filtering
    can_->CCCR &= ~FDCAN_CCCR_INIT; // Leave init mode
    uint32_t while_start_ms = AP_HAL::millis();
    while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
        if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
            return false;
        }
    }
    initialised_ = true;
    return true;
#else
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
#if defined(STM32G4)
        can_->RXGFC |= 0x2; //Reject All Standard ID Frames
#else
        can_->GFC |= 0x2; //Reject All Standard ID Frames
#endif
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
#endif // AP_Periph, STM32G4
}

uint16_t CANIface::getNumFilters() const
{
    return MAX_FILTER_LIST_SIZE;
}

bool CANIface::clock_init_ = false;
bool CANIface::init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode)
{
    Debug("Bitrate %lu mode %d", static_cast<unsigned long>(bitrate), static_cast<int>(mode));
    if (self_index_ > HAL_NUM_CAN_IFACES) {
        Debug("CAN drv init failed");
        return false;
    }
    if (can_ifaces[self_index_] == nullptr) {
        can_ifaces[self_index_] = this;
#if !defined(HAL_BOOTLOADER_BUILD)
        AP_HAL::get_HAL_mutable().can[self_index_] = this;
#endif
    }

    bitrate_ = bitrate;
    mode_ = mode;
    //Only do it once
    //Doing it second time will reset the previously initialised bus
    if (!clock_init_) {
        CriticalSectionLocker lock;
#if defined(STM32G4)
        RCC->APB1ENR1  |= RCC_APB1ENR1_FDCANEN;
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_FDCANRST;
        RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_FDCANRST;
#else
        RCC->APB1HENR  |= RCC_APB1HENR_FDCANEN;
        RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
        RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;
#endif
        clock_init_ = true;
    }

    /*
     * IRQ
     */
    if (!irq_init_) {
        CriticalSectionLocker lock;
        switch (can_interfaces[self_index_]) {
        case 0:
            nvicEnableVector(FDCAN1_IT0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(FDCAN1_IT1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            break;
#ifdef FDCAN2
        case 1:
            nvicEnableVector(FDCAN2_IT0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(FDCAN2_IT1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            break;
#endif
#ifdef FDCAN3
        case 2:
            nvicEnableVector(FDCAN3_IT0_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            nvicEnableVector(FDCAN3_IT1_IRQn, CORTEX_MAX_KERNEL_PRIORITY);
            break;
#endif
        }
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
    _bitrate = bitrate;
    Debug("Timings: presc=%u sjw=%u bs1=%u bs2=%u\n",
          unsigned(timings.prescaler), unsigned(timings.sjw), unsigned(timings.bs1), unsigned(timings.bs2));

    //setup timing register
    can_->NBTP = (((timings.sjw-1) << FDCAN_NBTP_NSJW_Pos)   |
                  ((timings.bs1-1) << FDCAN_NBTP_NTSEG1_Pos) |
                  ((timings.bs2-1) << FDCAN_NBTP_NTSEG2_Pos)  |
                  ((timings.prescaler-1) << FDCAN_NBTP_NBRP_Pos));

    if (fdbitrate) {
        if (!computeFDTimings(fdbitrate, fdtimings)) {
            can_->CCCR &= ~FDCAN_CCCR_INIT;
            uint32_t while_start_ms = AP_HAL::millis();
            while ((can_->CCCR & FDCAN_CCCR_INIT) == 1) {
                if ((AP_HAL::millis() - while_start_ms) > REG_SET_TIMEOUT) {
                    return false;
                }
            }
            return false;
        }
        _fdbitrate = fdbitrate;
        Debug("CANFD Timings: presc=%u bs1=%u bs2=%u\n",
              unsigned(fdtimings.prescaler), unsigned(fdtimings.bs1), unsigned(fdtimings.bs2));
        can_->DBTP = (((fdtimings.bs1-1) << FDCAN_DBTP_DTSEG1_Pos) |
                      ((fdtimings.bs2-1) << FDCAN_DBTP_DTSEG2_Pos)  |
                      ((fdtimings.prescaler-1) << FDCAN_DBTP_DBRP_Pos) |
                      ((fdtimings.sjw-1) << FDCAN_DBTP_DSJW_Pos)) |
            FDCAN_DBTP_TDC;
        // use a transmitter delay compensation offset of 10, suitable
        // for MCP2557FD transceiver with delay of 120ns
        can_->TDCR = 10<<FDCAN_TDCR_TDCO_Pos;
    }

    //RX Config
#if defined(STM32H7)
    can_->RXESC = 0; //Set for 8Byte Frames
#endif

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
#if defined(STM32G4)
    can_->ILS = FDCAN_ILS_PERR | FDCAN_ILS_SMSG;
#else
    can_->ILS = FDCAN_ILS_TCL | FDCAN_ILS_BOE;  //Set Line 1 for Transmit Complete Event Interrupt and Bus Off Interrupt
#endif
    // And Busoff error
#if defined(STM32G4)
    can_->TXBTIE = 0x7;
#else
    can_->TXBTIE = 0xFFFFFFFF;
#endif
    can_->ILE = 0x3;

#if HAL_CANFD_SUPPORTED
    can_->CCCR |= FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE; // enable sending CAN FD frames, and Bitrate switching
#endif

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
#if defined(STM32G4)
    const uint32_t base = SRAMCAN_BASE + FDCAN_MESSAGERAM_STRIDE * can_interfaces[self_index_];
    memset((void*)base, 0, FDCAN_MESSAGERAM_STRIDE);
    MessageRam_.StandardFilterSA = base;
    MessageRam_.ExtendedFilterSA = base + FDCAN_EXFILTER_OFFSET;
    MessageRam_.RxFIFO0SA = base + FDCAN_RXFIFO0_OFFSET;
    MessageRam_.RxFIFO1SA = base + FDCAN_RXFIFO1_OFFSET;
    MessageRam_.TxFIFOQSA = base + FDCAN_TXFIFO_OFFSET;

    can_->TXBC = 0; // fifo mode
#else
    uint32_t num_elements = 0;

    can_->RXESC = 0x777; //Support upto 64byte long frames
    can_->TXESC = 0x7; //Support upto 64byte long frames
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
#endif
}

void CANIface::handleTxCompleteInterrupt(const uint64_t timestamp_us)
{
    for (uint8_t i = 0; i < NumTxMailboxes; i++) {
        if ((can_->TXBTO & (1UL << i))) {

            if (!pending_tx_[i].pushed) {
                stats.tx_success++;
                stats.last_transmit_us = timestamp_us;
                if (pending_tx_[i].canfd_frame) {
                    stats.fdf_tx_success++;
                }
                pending_tx_[i].pushed = true;
            } else {
                continue;
            }

            if (pending_tx_[i].loopback && had_activity_) {
                CanRxItem rx_item;
                rx_item.frame = pending_tx_[i].frame;
                rx_item.timestamp_us = timestamp_us;
                rx_item.flags = AP_HAL::CANIface::Loopback;
                add_to_rx_queue(rx_item);
            }
            stats.num_events++;
            if (sem_handle != nullptr) {
                sem_handle->signal_ISR();
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
#if !defined(STM32G4)
        //Check if RAM allocated to RX FIFO
        if ((can_->RXF0C & FDCAN_RXF0C_F0S) == 0) {
            return false;
        }
#endif
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
#if !defined(STM32G4)
        //Check if RAM allocated to RX FIFO
        if ((can_->RXF1C & FDCAN_RXF1C_F1S) == 0) {
            return false;
        }
#endif
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
    AP_HAL::CANFrame frame {};
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

    if (frame_ptr[1] & FDF) {
        frame.setCanFD(true);
        stats.fdf_rx_received++;
    } else {
        frame.setCanFD(false);
    }

    frame.dlc = (frame_ptr[1] & DLC_MASK) >> 16;
    uint8_t *data = (uint8_t*)&frame_ptr[2];

    for (uint8_t i = 0; i < AP_HAL::CANFrame::dlcToDataLength(frame.dlc); i++) {
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
    if (add_to_rx_queue(rx_item)) {
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
    stats.num_events++;
    if (sem_handle != nullptr) {
        sem_handle->signal_ISR();
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
        stats.ecr = can_->ECR;
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
#if !defined(STM32G4)
    //Check if Tx FIFO is allocated
    if ((can_->TXBC & FDCAN_TXBC_TFQS) == 0) {
        return false;
    }
#endif
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

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *handle)
{
    sem_handle = handle;
    return true;
}

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
    uint64_t time = AP_HAL::micros64();

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
        if (sem_handle == nullptr) {
            break;
        }
        IGNORE_RETURN(sem_handle->wait(blocking_deadline - time)); // Block until timeout expires or any iface updates
        checkAvailable(read, write, pending_tx);  // Check what we got
        if ((read && in_read) || (write && in_write)) {
            return true;
        }
        time = AP_HAL::micros64();
    }
    return false;
}

#if !defined(HAL_BOOTLOADER_BUILD)
void CANIface::get_stats(ExpandingString &str)
{
    CriticalSectionLocker lock;
    str.printf("------- Clock Config -------\n"
               "CAN_CLK_FREQ:   %luMHz\n"
               "Std Timings: bitrate=%lu presc=%u\n"
               "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
               "FD Timings:  bitrate=%lu presc=%u\n"
               "sjw=%u bs1=%u bs2=%u sample_point=%f%%\n"
               "------- CAN Interface Stats -------\n"
               "tx_requests:    %lu\n"
               "tx_rejected:    %lu\n"
               "tx_overflow:    %lu\n"
               "tx_success:     %lu\n"
               "tx_timedout:    %lu\n"
               "tx_abort:       %lu\n"
               "rx_received:    %lu\n"
               "rx_overflow:    %lu\n"
               "rx_errors:      %lu\n"
               "num_busoff_err: %lu\n"
               "num_events:     %lu\n"
               "ECR:            %lx\n"
               "fdf_rx:         %lu\n"
               "fdf_tx_req:     %lu\n"
               "fdf_tx:         %lu\n",
               STM32_FDCANCLK/1000000UL,
               _bitrate, unsigned(timings.prescaler),
               unsigned(timings.sjw), unsigned(timings.bs1),
               unsigned(timings.bs2), timings.sample_point_permill/10.0f,
               _fdbitrate, unsigned(fdtimings.prescaler),
               unsigned(fdtimings.sjw), unsigned(fdtimings.bs1),
               unsigned(fdtimings.bs2), fdtimings.sample_point_permill/10.0f,
               stats.tx_requests,
               stats.tx_rejected,
               stats.tx_overflow,
               stats.tx_success,
               stats.tx_timedout,
               stats.tx_abort,
               stats.rx_received,
               stats.rx_overflow,
               stats.rx_errors,
               stats.num_busoff_err,
               stats.num_events,
               stats.ecr,
               stats.fdf_rx_received,
               stats.fdf_tx_requests,
               stats.fdf_tx_success);
}
#endif

/*
 * Interrupt handlers
 */
extern "C"
{
#ifdef HAL_CAN_IFACE1_ENABLE
    // FDCAN1
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
#endif

#ifdef HAL_CAN_IFACE2_ENABLE
    // FDCAN2
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
#endif

#ifdef HAL_CAN_IFACE3_ENABLE
    // FDCAN3
    CH_IRQ_HANDLER(FDCAN3_IT0_IRQHandler);
    CH_IRQ_HANDLER(FDCAN3_IT0_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(2, 0);
        CH_IRQ_EPILOGUE();
    }

    CH_IRQ_HANDLER(FDCAN3_IT1_IRQHandler);
    CH_IRQ_HANDLER(FDCAN3_IT1_IRQHandler)
    {
        CH_IRQ_PROLOGUE();
        handleCANInterrupt(2, 1);
        CH_IRQ_EPILOGUE();
    }
#endif
    
} // extern "C"

#endif //defined(STM32H7XX) || defined(STM32G4)

#endif //HAL_NUM_CAN_IFACES
