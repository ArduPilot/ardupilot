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

#if HAL_WITH_UAVCAN

#include "CANThread.h"
#include "fdcan.hpp"

class SLCANRouter;

namespace ChibiOS_CAN
{
/**
 * Driver error codes.
 * These values can be returned from driver functions negated.
 */
//static const uavcan::int16_t ErrUnknown               = 1000; ///< Reserved for future use
static const uavcan::int16_t ErrNotImplemented          = 1001; ///< Feature not implemented
static const uavcan::int16_t ErrInvalidBitRate          = 1002; ///< Bit rate not supported
static const uavcan::int16_t ErrLogic                   = 1003; ///< Internal logic error
static const uavcan::int16_t ErrUnsupportedFrame        = 1004; ///< Frame not supported (e.g. RTR, CAN FD, etc)
static const uavcan::int16_t ErrMsrInakNotSet           = 1005; ///< INAK bit of the MSR register is not 1
static const uavcan::int16_t ErrMsrInakNotCleared       = 1006; ///< INAK bit of the MSR register is not 0
static const uavcan::int16_t ErrBitRateNotDetected      = 1007; ///< Auto bit rate detection could not be finished
static const uavcan::int16_t ErrFilterNumConfigs        = 1008; ///< Number of filters is more than supported

/**
 * RX queue item.
 * The application shall not use this directly.
 */
struct CanRxItem {
    uavcan::uint64_t utc_usec;
    uavcan::CanFrame frame;
    uavcan::CanIOFlags flags;
    CanRxItem()
        : utc_usec(0)
        , flags(0)
    { }
};

/**
 * Single CAN iface.
 * The application shall not use this directly.
 */
class CanIface : public uavcan::ICanIface, uavcan::Noncopyable
{
#if AP_UAVCAN_SLCAN_ENABLED
    friend class ::SLCANRouter;
    static SLCANRouter _slcan_router;
#endif
    class RxQueue
    {
        CanRxItem* const buf_;
        const uavcan::uint8_t capacity_;
        uavcan::uint8_t in_;
        uavcan::uint8_t out_;
        uavcan::uint8_t len_;
        uavcan::uint32_t overflow_cnt_;
        void registerOverflow();

    public:
        RxQueue(CanRxItem* buf, uavcan::uint8_t capacity)
            : buf_(buf)
            , capacity_(capacity)
            , in_(0)
            , out_(0)
            , len_(0)
            , overflow_cnt_(0)
        { }

        void push(const uavcan::CanFrame& frame, const uint64_t& utc_usec, uavcan::CanIOFlags flags);
        void pop(uavcan::CanFrame& out_frame, uavcan::uint64_t& out_utc_usec, uavcan::CanIOFlags& out_flags);

        void reset();

        unsigned getLength() const
        {
            return len_;
        }

        uavcan::uint32_t getOverflowCount() const
        {
            return overflow_cnt_;
        }
    };

    struct MessageRAM {
        uavcan::uint32_t StandardFilterSA;
        uavcan::uint32_t ExtendedFilterSA;
        uavcan::uint32_t RxFIFO0SA;
        uavcan::uint32_t RxFIFO1SA;
        uavcan::uint32_t TxFIFOQSA;
        uavcan::uint32_t EndAddress;
    } MessageRam_;

    struct Timings {
        uavcan::uint16_t prescaler;
        uavcan::uint8_t sjw;
        uavcan::uint8_t bs1;
        uavcan::uint8_t bs2;

        Timings()
            : prescaler(0)
            , sjw(0)
            , bs1(0)
            , bs2(0)
        { }
    };

    struct TxItem {
        uavcan::MonotonicTime deadline;
        uavcan::CanFrame frame;
        bool loopback;
        bool abort_on_error;
        uint8_t index;
        TxItem() :
            loopback(false),
            abort_on_error(false)
        { }
    };

    enum { NumTxMailboxes = 32 };

    static const uavcan::uint32_t TSR_ABRQx[NumTxMailboxes];
    static uint32_t FDCANMessageRAMOffset_;
    RxQueue rx_queue_;
    fdcan::CanType* const can_;
    uavcan::uint64_t error_cnt_;
    uavcan::uint32_t served_aborts_cnt_;
    BusEvent& update_event_;
    TxItem pending_tx_[NumTxMailboxes];
    uavcan::uint8_t peak_tx_mailbox_index_;
    const uavcan::uint8_t self_index_;
    bool had_activity_;
    int computeTimings(uavcan::uint32_t target_bitrate, Timings& out_timings);

    virtual uavcan::int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline,
                                 uavcan::CanIOFlags flags) override;

    virtual uavcan::int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic,
                                    uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags) override;

    virtual uavcan::int16_t configureFilters(const uavcan::CanFilterConfig* filter_configs,
            uavcan::uint16_t num_configs) override;

    virtual uavcan::uint16_t getNumFilters() const override;

    void setupMessageRam(void);

    static uint32_t FDCAN2MessageRAMOffset_;
public:
    enum { MaxRxQueueCapacity = 254 };

    enum OperatingMode {
        NormalMode,
        SilentMode
    };

    CanIface(fdcan::CanType* can, BusEvent& update_event, uavcan::uint8_t self_index,
             CanRxItem* rx_queue_buffer, uavcan::uint8_t rx_queue_capacity);

    /**
     * Initializes the hardware CAN controller.
     * Assumes:
     *   - Iface clock is enabled
     *   - Iface has been resetted via RCC
     *   - Caller will configure NVIC by itself
     */
    int init(const uavcan::uint32_t bitrate, const OperatingMode mode);

    void handleTxCompleteInterrupt(uavcan::uint64_t utc_usec);
    void handleRxInterrupt(uavcan::uint8_t fifo_index);

    bool readRxFIFO(uavcan::uint8_t fifo_index);
    /**
     * This method is used to count errors and abort transmission on error if necessary.
     * This functionality used to be implemented in the SCE interrupt handler, but that approach was
     * generating too much processing overhead, especially on disconnected interfaces.
     *
     * Should be called from RX ISR, TX ISR, and select(); interrupts must be enabled.
     */
    void pollErrorFlagsFromISR();

    void discardTimedOutTxMailboxes(uavcan::MonotonicTime current_time);

    bool canAcceptNewTxFrame(const uavcan::CanFrame& frame) const;
    bool isRxBufferEmpty() const;

    /**
     * Number of RX frames lost due to queue overflow.
     * This is an atomic read, it doesn't require a critical section.
     */
    uavcan::uint32_t getRxQueueOverflowCount() const
    {
        return rx_queue_.getOverflowCount();
    }

    /**
     * Total number of hardware failures and other kinds of errors (e.g. queue overruns).
     * May increase continuously if the interface is not connected to the bus.
     */
    virtual uavcan::uint64_t getErrorCount() const override;

    /**
     * Number of times the driver exercised library's requirement to abort transmission on first error.
     * This is an atomic read, it doesn't require a critical section.
     * See @ref uavcan::CanIOFlagAbortOnError.
     */
    uavcan::uint32_t getVoluntaryTxAbortCount() const
    {
        return served_aborts_cnt_;
    }

    /**
     * Returns the number of frames pending in the RX queue.
     * This is intended for debug use only.
     */
    unsigned getRxQueueLength() const;

    /**
     * Whether this iface had at least one successful IO since the previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();

    /**
     * Peak number of TX mailboxes used concurrently since initialization.
     * Range is [1, 3].
     * Value of 3 suggests that priority inversion could be taking place.
     */
    uavcan::uint8_t getPeakNumTxMailboxesUsed() const
    {
        return uavcan::uint8_t(peak_tx_mailbox_index_ + 1);
    }

    fdcan::CanType* can_reg(void)
    {
        return can_;
    }

#if AP_UAVCAN_SLCAN_ENABLED
    static SLCANRouter &slcan_router() { return _slcan_router; }
#endif
};

/**
 * CAN driver, incorporates all available CAN ifaces.
 * Please avoid direct use, prefer @ref CanInitHelper instead.
 */
class CanDriver : public uavcan::ICanDriver, uavcan::Noncopyable
{
    BusEvent update_event_;
    static bool clock_init_;
    CanIface if0_;
#if UAVCAN_STM32_NUM_IFACES > 1
    CanIface if1_;
#endif
    bool initialized_by_me_[UAVCAN_STM32_NUM_IFACES];
    uavcan::uint8_t num_ifaces_;
    uavcan::uint8_t if_int_to_gl_index_[UAVCAN_STM32_NUM_IFACES];


    virtual uavcan::int16_t select(uavcan::CanSelectMasks& inout_masks,
                                   const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces],
                                   uavcan::MonotonicTime blocking_deadline) override;

    static void initOnce();
    static void initOnce(uavcan::uint8_t can_number, bool enable_irqs);

public:
    template <unsigned RxQueueCapacity>
    CanDriver(CanRxItem(&rx_queue_storage)[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity])
        : update_event_(*this)
        , if0_(fdcan::Can[0], update_event_, 0, rx_queue_storage[0], RxQueueCapacity)
#if UAVCAN_STM32_NUM_IFACES > 1
        , if1_(fdcan::Can[1], update_event_, 1, rx_queue_storage[1], RxQueueCapacity)
#endif
    {
        uavcan::StaticAssert<(RxQueueCapacity <= CanIface::MaxRxQueueCapacity)>::check();
    }

    /**
     * This function returns select masks indicating which interfaces are available for read/write.
     */
    uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (& pending_tx)[uavcan::MaxCanIfaces]) const;

    BusEvent* getUpdateEvent()
    {
        return &update_event_;
    }
    /**
     * Whether there's at least one interface where receive() would return a frame.
     */
    bool hasReadableInterfaces() const;

    /**
     * Returns zero if OK.
     * Returns negative value if failed (e.g. invalid bitrate).
     */
    int init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode);
    int init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode, uavcan::uint8_t can_number);

    virtual CanIface* getIface(uavcan::uint8_t iface_index) override;

    virtual uavcan::uint8_t getNumIfaces() const override
    {
        return num_ifaces_;
    }

    /**
     * Whether at least one iface had at least one successful IO since previous call of this method.
     * This is designed for use with iface activity LEDs.
     */
    bool hadActivity();
};

/**
 * Helper class.
 * Normally only this class should be used by the application.
 * 145 usec per Extended CAN frame @ 1 Mbps, e.g. 32 RX slots * 145 usec --> 4.6 msec before RX queue overruns.
 */
template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
    CanRxItem queue_storage_[UAVCAN_STM32_NUM_IFACES][RxQueueCapacity];

public:
    enum { BitRateAutoDetect = 0 };

    CanDriver driver;

    CanInitHelper() :
        driver(queue_storage_)
    { }

    /**
     * This overload simply configures the provided bitrate.
     * Auto bit rate detection will not be performed.
     * Bitrate value must be positive.
     * @return  Negative value on error; non-negative on success. Refer to constants Err*.
     */
    int init(uavcan::uint32_t bitrate)
    {
        return driver.init(bitrate, CanIface::NormalMode);
    }

    int init(const uavcan::uint32_t bitrate, const CanIface::OperatingMode mode, uavcan::uint8_t can_number)
    {
        return driver.init(bitrate, mode, can_number);
    }

    /**
     * This function can either initialize the driver at a fixed bit rate, or it can perform
     * automatic bit rate detection. For theory please refer to the CiA application note #801.
     *
     * @param delay_callable    A callable entity that suspends execution for strictly more than one second.
     *                          The callable entity will be invoked without arguments.
     *                          @ref getRecommendedListeningDelay().
     *
     * @param inout_bitrate     Fixed bit rate or zero. Zero invokes the bit rate detection process.
     *                          If auto detection was used, the function will update the argument
     *                          with established bit rate. In case of an error the value will be undefined.
     *
     * @return                  Negative value on error; non-negative on success. Refer to constants Err*.
     */
    template <typename DelayCallable>
    int init(DelayCallable delay_callable, uavcan::uint32_t& inout_bitrate = BitRateAutoDetect)
    {
        if (inout_bitrate > 0) {
            return driver.init(inout_bitrate, CanIface::NormalMode);
        } else {
            static const uavcan::uint32_t StandardBitRates[] = {
                1000000,
                500000,
                250000,
                125000
            };

            for (uavcan::uint8_t br = 0; br < sizeof(StandardBitRates) / sizeof(StandardBitRates[0]); br++) {
                inout_bitrate = StandardBitRates[br];

                const int res = driver.init(inout_bitrate, CanIface::SilentMode);

                delay_callable();

                if (res >= 0) {
                    for (uavcan::uint8_t iface = 0; iface < driver.getNumIfaces(); iface++) {
                        if (!driver.getIface(iface)->isRxBufferEmpty()) {
                            // Re-initializing in normal mode
                            return driver.init(inout_bitrate, CanIface::NormalMode);
                        }
                    }
                }
            }

            return -ErrBitRateNotDetected;
        }
    }

    /**
     * Use this value for listening delay during automatic bit rate detection.
     */
    static uavcan::MonotonicDuration getRecommendedListeningDelay()
    {
        return uavcan::MonotonicDuration::fromMSec(1050);
    }
};

}

#include "CANSerialRouter.h"

#endif //HAL_WITH_UAVCAN
